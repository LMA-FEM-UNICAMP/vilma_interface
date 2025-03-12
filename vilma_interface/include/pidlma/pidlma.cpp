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

PIDLMA::PIDLMA()
{
    configure(0.0, 0.0, 0.0, 0.0, 3.0);
}

void PIDLMA::configure(double k_p, double k_d, double k_i, double t, double ramp_rate)
{
    kp_ = k_p;
    kd_ = k_d;
    ki_ = k_i;
    error_ant_ = 0;
    error_sum_ = 0;
    u_ = 0;
    t_ant_ = t;
    velocity_reference_in_ramp_ = 0;
}

void PIDLMA::reset()
{
    error_ant_ = 0;
    error_sum_ = 0;
}

double PIDLMA::calculate(double value, double reference, double t)
{
    double dt = t - t_ant_;

    update_velocity_reference_in_ramp(reference, dt);

    double error = velocity_reference_in_ramp_ - value;

    error_sum_ += (u_ >= 1.0 || u_ <= -1.0) ? 0 : error * dt;

    u_ = error * kp_ + kd_ * (error - error_ant_) / dt + ki_ * error_sum_;

    u_ = (u_ > 1.0) ? 1.0 : ((u_ < -1.0) ? -1.0 : u_);

    error_ant_ = error;

    return u_;
}

void PIDLMA::update_velocity_reference_in_ramp(double velocity_target, double dt)
{
    // If velocity reference is below target velocity
    if (velocity_target > velocity_reference_in_ramp_)
    {
        // Increment velocity reference in ramp with a ratio of 3
        velocity_reference_in_ramp_ += ramp_rate_ * dt;

        // Saturate the value if velocity reference transpass velocity target
        velocity_reference_in_ramp_ = (velocity_reference_in_ramp_ > velocity_target) ? velocity_target : velocity_reference_in_ramp_;
    }
    // If velocity reference is above target velocity
    else if (velocity_target < velocity_reference_in_ramp_)
    {
        // Decrement velocity reference in ramp with a ratio of 3
        velocity_reference_in_ramp_ -= ramp_rate_ * dt;

        // Saturate the value if velocity reference transpass velocity target
        velocity_reference_in_ramp_ = (velocity_reference_in_ramp_ < velocity_target) ? velocity_target : velocity_reference_in_ramp_;
    }
}