/*
 * pidlma.cpp
 *
 *  Created on: Nov 6, 2015
 *      Author: Olmer
 *  Updated on: Mar 11, 2024
 *      Maintainer: Toffanetto
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "pidlma/pidlma.hpp"

#include <iostream>
#include <algorithm>

PIDLMA::PIDLMA()
{
  // configure(0.0, 0.0, 0.0, 10.0, 0.0, 3.0, -0.1);

  control_mode_ = INITIAL_MODE;
}

void PIDLMA::configure(const PIDLMA_config_t &control_configuration)
{
  kp_t_ = control_configuration.k_p_t;
  kd_t_ = control_configuration.k_d_t;
  ki_t_ = control_configuration.k_i_t;
  int_max_t_ = control_configuration.int_max_t;
  kp_b_ = control_configuration.k_p_b;
  kd_b_ = control_configuration.k_d_b;
  ki_b_ = control_configuration.k_i_b;
  int_max_b_ = control_configuration.int_max_b;
  ramp_rate_ = control_configuration.ramp_rate;
  brake_deadband_ = control_configuration.brake_deadband;
  output_max_ = control_configuration.output_max;
  output_min_ = control_configuration.output_min;
  t_ant_ = control_configuration.t;
  error_ant_ = 0;
  error_sum_ = 0;
  u_ = 0;
  velocity_reference_in_ramp_ = 0;
  maf_size_ = control_configuration.maf_size;
  max_brake_rate_ = control_configuration.max_brake_rate;
  max_throttle_rate_ = control_configuration.max_throttle_rate;

  kp_ = 0.0;
  kd_ = 0.0;
  ki_ = 0.0;
  int_max_ = 0.0;
  control_mode_ = INITIAL_MODE;
  integrating_ = true;
}

void PIDLMA::reset(int64_t t)
{
  t_ant_ = t;
  error_ant_ = 0;
  error_sum_ = 0;
  u_ = 0;
  velocity_reference_in_ramp_ = 0;
}

void PIDLMA::calculate(LongActuationCommand &control_action, int64_t t)
{

  // Output rate limit assessing variables
  double brake_change = 0.0;
  double max_brake_change = 0.0;
  double max_throttle_change = 0.0;
  double throttle_change = 0.0;

  // Real sample period in seconds
  double dt = static_cast<double>(t - t_ant_) * 1e-9;

  // Update current time for next period calculation
  t_ant_ = t;

  // Update velocity reference in ramp
  update_velocity_reference_in_ramp(reference_, dt);

  // Calculate error
  double error = velocity_reference_in_ramp_ - maf_longitudinal_speed_output_;

  if (error_ant_ == 0.0)
  {
    error_ant_ = error;
  }

  // Set controller
  if (error > 0.0 && control_mode_ != THROTTLE_MODE)
  {
    kp_ = kp_t_;
    kd_ = kd_t_;
    ki_ = ki_t_;
    int_max_ = int_max_t_;
    error_sum_ = 0;
    control_mode_ = THROTTLE_MODE;
  }
  else if (error < 0.0 && control_mode_ != BRAKING_MODE)
  {
    kp_ = kp_b_;
    kd_ = kd_b_;
    ki_ = ki_b_;
    int_max_ = int_max_b_;
    error_sum_ = 0;
    control_mode_ = BRAKING_MODE;
  }

  // Integrate error and saturate if greater then int_max_ or output is saturated
  error_sum_ += (integrating_) ? error * dt : 0.0;
  integrating_ = true; // Resetting anti-wind up flag

  // Compute control action
  double p = error * kp_;
  double i = ki_ * error_sum_;
  double d = kd_ * (error - error_ant_) / dt;
  u_ = p + i + d;

  // Debug
  control_action.p = p;
  control_action.i = i;
  control_action.d = d;
  control_action.dt = dt;
  control_action.e = error;
  control_action.e_i = error_sum_;
  control_action.u = u_;
  control_action.ref = velocity_reference_in_ramp_;
  control_action.v = maf_longitudinal_speed_output_;

  // Saturate control action
  u_ = (u_ > output_max_) ? output_max_ : ((u_ < output_min_) ? output_min_ : u_);

  // Save current error for compute derivative in next step
  error_ant_ = error;

  //* Checking control action value to assign as braking, accelerating or engine braking
  if (u_ <= brake_deadband_) /// Active braking
  {
    //* Assign the control action as braking percentage mapped from [-1.0, -0.1] to [0.0, 1.0]

    brake_change = (-u_ + brake_deadband_) / (1.0 - brake_deadband_) - control_action_prev_.brake_value;

    // Sometimes that you brake hard and fast, the ECU presents some issues, maybe
    // a ramp to filter the brake_value would be good.
    // TODO: Test
    max_brake_change = max_brake_rate_ / 100.0 * dt;

    control_action.brake_value = control_action_prev_.brake_value + std::clamp(brake_change, -max_brake_change, max_brake_change);

    //* Setting brake mode in autonomous
    control_action.brake_command = static_cast<double>(JoystickMA::BRAKE_COMMAND_AUTO);
  }
  else if (u_ >= 0) /// Accelerating
  {
    //* Assign control action as gas pedal position with limited rate [0.0, 1.0]

    max_throttle_change = max_throttle_rate_ / 100.0 * dt;

    throttle_change = u_ - control_action_prev_.gas_value;

    control_action.gas_value = control_action_prev_.gas_value + std::clamp(throttle_change, -max_throttle_change, max_throttle_change);
  }
  /// Else: engine braking

  /// Special case: Keep vehicle stopped
  if (reference_ == 0.0 && maf_longitudinal_speed_output_ <= 0.1)
  {
    //* Assign Full brake
    control_action.brake_value = 1.0;

    //* Setting brake mode in autonomous
    control_action.brake_command = static_cast<double>(JoystickMA::BRAKE_COMMAND_AUTO);

    //* Double check that the control is not throttling
    control_action.gas_value = 0.0;
  }

  control_action_prev_ = control_action;

  // Setting anti-windup flag to next loop
  if ((u_ >= output_max_ && error > 0) ||             // Top output saturation
      (u_ <= output_min_ && error < 0) ||             // Bottom output saturation
      (error_sum_ >= int_max_ && error > 0) ||        // Top integrator value saturation
      (error_sum_ <= -int_max_ && error < 0) ||       // Bottom integrator value saturation
      (abs(throttle_change) > max_throttle_change) || // Throttle rate limit reached
      (abs(brake_change) > max_brake_change))         // Braking rate limit reached
  {
    integrating_ = false; // Disable integration
  }
}

void PIDLMA::update_velocity_reference_in_ramp(double velocity_target, double dt)
{
  //* In Autoware, don't need to filter the reference
  if (ramp_rate_ == 0.0)
  {
    velocity_reference_in_ramp_ = velocity_target;
    return;
  }

  // If velocity reference is below target velocity
  if (velocity_target > velocity_reference_in_ramp_)
  {
    // Increment velocity reference in ramp with a ratio of 3
    velocity_reference_in_ramp_ += ramp_rate_ * dt;

    // Saturate the value if velocity reference transpass velocity target
    velocity_reference_in_ramp_ =
        (velocity_reference_in_ramp_ > velocity_target) ? velocity_target : velocity_reference_in_ramp_;
  }
  // If velocity reference is above target velocity
  else if (velocity_target < velocity_reference_in_ramp_)
  {
    // Decrement velocity reference in ramp with a ratio of 3
    velocity_reference_in_ramp_ -= ramp_rate_ * dt;

    // Saturate the value if velocity reference transpass velocity target
    velocity_reference_in_ramp_ =
        (velocity_reference_in_ramp_ < velocity_target) ? velocity_target : velocity_reference_in_ramp_;
  }
}

void PIDLMA::updateVelocityFilter(double longitudinal_speed)
{

  if (maf_longitudinal_speed_buffer_.size() == 0)
  {
    maf_longitudinal_speed_output_ = 0;
  }
  else if (maf_longitudinal_speed_buffer_.size() < maf_size_)
  {
    maf_longitudinal_speed_buffer_.push(longitudinal_speed);
    maf_longitudinal_speed_output_ = maf_longitudinal_speed_output_ + longitudinal_speed / maf_longitudinal_speed_buffer_.size() - maf_longitudinal_speed_buffer_.front() / maf_longitudinal_speed_buffer_.size();
  }
  else if (maf_longitudinal_speed_buffer_.size() == maf_size_)
  {
    maf_longitudinal_speed_buffer_.push(longitudinal_speed);
    maf_longitudinal_speed_output_ = maf_longitudinal_speed_output_ + longitudinal_speed / maf_size_ - maf_longitudinal_speed_buffer_.front() / maf_size_;
    maf_longitudinal_speed_buffer_.pop();
  }
}

void PIDLMA::setReference(double reference)
{
  this->reference_ = reference;

  if (reference == 0.0)
  {
    velocity_reference_in_ramp_ = 0.0;
  }
}