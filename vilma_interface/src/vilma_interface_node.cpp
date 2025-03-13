/*
 * vilma_interface_node.cpp
 *
 *  Created on: Mar 12, 2025
 *
 *  Author: Gabriel Toffanetto França da Rocha
 *
 *  Laboratory of Autonomous Mobility (LMA)
 *  School of Mechanical Engineering (FEM)
 *  University of Campinas (Unicamp)
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

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "vilma_interface/vilma_interface.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto vilma_interface_node = std::make_shared<vilma::VilmaInterface>();

    //* Creating multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor mt_executor;

    //* Adding node to executor
    mt_executor.add_node(vilma_interface_node);

    RCLCPP_INFO(vilma_interface_node->get_logger(), "Starting VILMA's vehicle interface through UDP");
    mt_executor.spin();
    RCLCPP_INFO(vilma_interface_node->get_logger(), "Shutting down VILMA's vehicle interface. \n");

    rclcpp::shutdown();
    return 0;
}
