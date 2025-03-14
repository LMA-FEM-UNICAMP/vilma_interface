/*
 * vilma_interface.hpp
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

#ifndef vilma_interface__HPP_
#define vilma_interface__HPP_

// C++ includes

#include <algorithm>
#include <functional>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>

// ROS includes

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/header.hpp"

/* Autoware includes */

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>

// Personal libraries includes

#include "vilma_interface/vilma_ma_labeling.hpp"
#include "pidlma/pidlma.hpp"
#include "maudp/maudp.h"

namespace vilma
{
    class VilmaInterface : public rclcpp::Node
    {

    public:
        VilmaInterface();
        ~VilmaInterface();

    private:
        // Parameters

        // Attributes

        microautobox::maudp ma_udp_client;
        std::mutex mutex_joystick_command_;
        std::vector<double> joystick_command_;
        std::vector<double> to_ma_vector_;
        std::vector<double> from_ma_vector_;
        int to_ma_length_;
        int from_ma_length_;

        int ma_operation_mode_;
        uint8_t vilma_control_mode_;
        bool change_control_mode_enabled_;

        /* Vehicle */
        double brake_deadband_;
        double max_steering_tire_angle_rad_;
        double max_gas_value_;
        double max_brake_value_;
        double max_speed_m_s_;
        double speed_reference_ramp_rate_;
        double brake_user_pressure_set_emergency_;
        bool autonomous_shift_enable_;

        PIDLMA velocity_controller_;

        /* Command validation */
        int autoware_command_time_validity_ms_;
        int communication_timeout_ms_;

        // Methods

        unsigned short to_ma();
        void from_ma(int type_tx, rclcpp::Time stamp);

        bool set_control_mode(uint8_t control_mode);

        double get_steering_value(double steering_tire_angle_rad);

        /// ROS 2

        // Timers

        rclcpp::Time ma_timer_last_stamp_;
        rclcpp::TimerBase::SharedPtr ma_timer_;
        rclcpp::TimerBase::SharedPtr ma_sleep_timer_;

        int ma_timer_period_ms_;
        int ma_sleep_period_min_;

        void ma_timer_callback();
        void ma_sleep_callback();

        // Autoware

        /* Subscribers */

        rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
        rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
        rclcpp::Subscription<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_sub_;

        /* Publishers */

        rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_pub_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_pub_;

        /* Services */

        rclcpp::Service<autoware_vehicle_msgs::srv::ControlModeCommand>::SharedPtr control_mode_request_server_;

        /* Callback groups */

        /// Using callback groups to parallel execution of node timers and subscriber callbacks

        rclcpp::CallbackGroup::SharedPtr timers_callback_group_;
        rclcpp::CallbackGroup::SharedPtr subscribers_callback_group_;

        /* Callbacks */

        void control_cmd_callback(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
        void gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
        void engage_callback(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
        void control_mode_request_callback(const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
                                           const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response);

        // VILMA

        /* Subscribers */

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_ma_sub_;

        /* Publishers */

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_ma_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sensors_ma_pub_;

        /* Messages */

        std_msgs::msg::Float64MultiArray state_ma_msg_;
        std_msgs::msg::Float64MultiArray sensors_ma_msg_;

        /* Callbacks */

        void joystick_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
    };

} // namespace vilma
#endif // vilma_interface__HPP_
