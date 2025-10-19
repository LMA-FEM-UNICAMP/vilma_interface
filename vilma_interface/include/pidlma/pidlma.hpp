/*
 * pidlma.hpp
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

#ifndef SRC_PIDLMA_HPP_
#define SRC_PIDLMA_HPP_


#include "vilma_interface/vilma_ma_labeling.hpp"

struct LongActuationCommand
{
  double brake_command = 0.0;
  double brake_value = 0.0;
  double gas_value = 0.0;
};

class PIDLMA
{
  double kp_, kd_, ki_;
  double error_ant_;
  double error_sum_;
  double t_ant_;
  double u_;
  double ramp_rate_;
  double velocity_reference_in_ramp_;
  double brake_deadband_;
  double reference_;

public:

  /**
   * @brief Construct a new PIDLMA object
   * 
   */
  PIDLMA();

  /**
   * @brief Construct a new PIDLMA object
   * 
   * @param ramp_rate 
   */
  PIDLMA(double ramp_rate);

  /**
   * @brief Configure the PID controller if the gains, initialize the time, configure reference integration
   * rate and the brake deadband for engine braking.
   * 
   * @param k_p 
   * @param k_d 
   * @param k_i 
   * @param t 
   * @param ramp_rate 
   * @param brake_deadband 
   */
  void configure(double k_p, double k_d, double k_i, double t, double ramp_rate, double brake_deadband);

  /**
   * @brief Reset integration buffers in the controller and time
   * 
   * @param t 
   */
  void reset(double t);

  /**
   * @brief Function that calculate the control action (throttle, braking and braking mode) for the current 
   * speed value at a time t.
   * 
   * @param control_action Returned control action in LongActuationCommand format
   * @param value Current longitudinal velocity 
   * @param t Current time in seconds
   */
  void calculate(LongActuationCommand &control_action, double value, double t);

  /**
   * @brief Set the Reference object
   * 
   * @param reference 
   */
  void setReference(double reference);

  /**
   * @brief Auxiliary function to integrate the reference in ramp to smooth the control.
   * 
   * @param velocity_target 
   * @param dt 
   */
  void update_velocity_reference_in_ramp(double velocity_target, double dt);
};

#endif /* SRC_PIDLMA_HPP_ */
