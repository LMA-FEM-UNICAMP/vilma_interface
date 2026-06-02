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

PIDLMA::PIDLMA()
{
  // configure(0.0, 0.0, 0.0, 10.0, 0.0, 3.0, -0.1);
}

void PIDLMA::configure(const PIDLMA_config_t& control_configuration)
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
}

void PIDLMA::reset(int64_t t)
{
  t_ant_ = t;
  error_ant_ = 0;
  error_sum_ = 0;
  velocity_reference_in_ramp_ = 0;
}

void PIDLMA::calculate(LongActuationCommand& control_action, double value, int64_t t)
{
  // Gains for calculate control action
  double kp_, kd_, ki_;
  double int_max_;

  // Real sample period in seconds
  double dt = static_cast<double>(t - t_ant_) * 1e-9;

  // Update current time for next period calculation
  t_ant_ = t;

  // Update velocity reference in ramp
  update_velocity_reference_in_ramp(reference_, dt);

  // Calculate error
  double error = velocity_reference_in_ramp_ - value;

  // Set integrator saturation
  // ? Same for throttle and braking?
  int_max_ = int_max_t_;

  // Integrate error and saturate if greater then int_max_ or output is saturated
  error_sum_ += ((u_ >= output_max_ && error > 0) || (u_ <= output_min_ && error < 0) ||
                 (error_sum_ >= int_max_ && error > 0) || (error_sum_ <= -int_max_ && error < 0)) ?
                    0 :
                    error * dt;

  //* Select the gains for positive error (throttle) or negative error (braking)
  if (error_sum_ >= 0.0)
  {
    // Throttle gains
    kp_ = kp_t_;
    kd_ = kp_t_;
    ki_ = kp_t_;
  }
  else
  {
    // Braking gains
    kp_ = kp_b_;
    kd_ = kp_b_;
    ki_ = kp_b_;
  }

  // Compute control action
  u_ = error * kp_ + kd_ * (error - error_ant_) / dt + ki_ * error_sum_;

  std::cout << "***u = " << u_ << std::endl;
  std::cout << "***p = " << error * kp_ << std::endl;
  std::cout << "***i = " << ki_ * error_sum_ << std::endl;

  // Saturate control action
  u_ = (u_ > output_max_) ? output_max_ : ((u_ < output_min_) ? output_min_ : u_);

  // Save current error for compute derivative in next step
  error_ant_ = error;

  //* Checking control action value to assign as braking, accelerating or engine braking
  if (u_ <= brake_deadband_)  /// Active braking
  {
    //* Assign the control action as braking percentage mapped from [-1.0, -0.1] to [0.0, 1.0]
    control_action.brake_value = (-u_ + brake_deadband_) / (1.0 - brake_deadband_);

    // TODO: Sometimes that you brake hard and fast, the ECU presents some issues, maybe
    // TODO:  a ramp to filter the brake_value would be good.

    //* Setting brake mode in autonomous
    control_action.brake_command = static_cast<double>(JoystickMA::BRAKE_COMMAND_AUTO);
  }
  else if (u_ >= 0)  /// Accelerating
  {
    //* Assign control action as gas pedal position [0.0, 1.0]
    control_action.gas_value = u_;  // + 0.07;
  }
  /// Else: engine braking

  /// Special case: Keep vehicle stopped
  if (reference_ == 0.0 && value <= 0.1)
  {
    //* Assign Full brake
    control_action.brake_value = 1.0;

    //* Setting brake mode in autonomous
    control_action.brake_command = static_cast<double>(JoystickMA::BRAKE_COMMAND_AUTO);

    //* Double check that the control is not throttling
    control_action.gas_value = 0.0;
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

void PIDLMA::setReference(double reference)
{
  this->reference_ = reference;

  if (reference == 0.0)
  {
    velocity_reference_in_ramp_ = 0.0;
  }
}