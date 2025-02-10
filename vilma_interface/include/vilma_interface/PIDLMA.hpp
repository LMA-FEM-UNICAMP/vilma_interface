/*
 * PIDLMA.hpp
 *
 *  Created on: Nov 6, 2015
 *      Author: Olmer
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

class PIDLMA
{
  double kp, kd, ki, kimax;
  double error_ant;
  double error_sum;

public:
  PIDLMA()
  {
    configure(0.0, 0.0, 0.0, 0.0);
  };

  void configure(double k_p, double k_d, double k_i, double k_i_max)
  {
    kp = k_p;
    kd = k_d;
    ki = k_i;
    kimax = k_i_max;
    error_ant = 0;
    error_sum = 0;
  };

  void reset()
  {
    error_ant = 0;
    error_sum = 0;
  }

  double calculate(double error, double dt)
  {
    double u;
    error_sum += error * dt;                                         // integral
    error_sum = std::max(-1.0 * kimax, std::min(error_sum, kimax));  // stauration integral

    u = error * kp + kd * (error - error_ant) / dt + ki * error_sum;
    error_ant = error;
    return u;
  };
};

#endif /* SRC_PIDLMA_HPP_ */
