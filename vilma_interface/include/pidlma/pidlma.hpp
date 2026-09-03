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
#include <cstdint>
#include <queue>

typedef struct
{
  double k_p_t;
  double k_d_t;
  double k_i_t;
  double int_max_t;
  double k_p_b;
  double k_d_b;
  double k_i_b;
  double int_max_b;
  double t;
  double ramp_rate;
  double brake_deadband;
  double output_min;
  double output_max;
  uint8_t maf_size;
  double max_brake_rate;
  double max_throttle_rate;
} PIDLMA_config_t;

struct LongActuationCommand
{
  double brake_command = 0.0;
  double brake_value = 0.0;
  double gas_value = 0.0;
  double u;
  double p;
  double i;
  double d;
  double e;
  double e_i;
  double dt;
  double ref;
  double v;
};

class PIDLMA
{

  constexpr static uint8_t INITIAL_MODE = -1;
  constexpr static uint8_t THROTTLE_MODE = 1;
  constexpr static uint8_t BRAKING_MODE = 2;

  double kp_t_, kd_t_, ki_t_;
  double int_max_t_;
  double kp_b_, kd_b_, ki_b_;
  double int_max_b_;
  double kp_;
  double kd_;
  double ki_;
  double int_max_;
  double error_ant_;
  double error_sum_;
  double t_ant_;
  double u_;
  double ramp_rate_;
  double velocity_reference_in_ramp_;
  double brake_deadband_;
  double reference_;
  double output_max_;
  double output_min_;

  bool integrating_;

  int8_t control_mode_;

  std::queue<double> maf_longitudinal_speed_buffer_;
  double maf_longitudinal_speed_output_;
  uint8_t maf_size_;

  double max_brake_rate_;
  double max_throttle_rate_;

  LongActuationCommand control_action_prev_;


public:
  /**
   * @brief Construct a new PIDLMA object
   *
   */
  PIDLMA();

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
   * @param output_min
   * @param output_max
   */
  void configure(const PIDLMA_config_t&);

  /**
   * @brief Reset integration buffers in the controller and time
   *
   * @param t Nanoseconds since epoch
   */
  void reset(int64_t t);

  /**
   * @brief Function that calculate the control action (throttle, braking and braking mode) for the current
   * speed value at a time t.
   *
   * @param control_action Returned control action in LongActuationCommand format
   * @param t Nanoseconds since epoch
   */
  void calculate(LongActuationCommand& control_action, int64_t t);

  /**
   * @brief Anti-aliasing filter for measured speed
   *
   * @param longitudinal_speed
   */
  void updateVelocityFilter(double longitudinal_speed);

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
