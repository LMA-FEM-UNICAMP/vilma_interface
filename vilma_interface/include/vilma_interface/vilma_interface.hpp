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

#include <mutex>
#include <atomic>

// ROS includes

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

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
#include "temp_condition_filter/temp_condition_filter.hpp"

namespace vilma
{

    class BeepOptions
{
public:
    constexpr static uint NONE = 0;
    constexpr static uint OK = 1;
    constexpr static uint ALERT = 2;
    constexpr static uint EMERGENCY = 3;
};

    class VilmaInterface : public rclcpp::Node
    {

    public:
        // Aliases

        /**
         * @brief Alias for control modes in autoware_vehicle_msgs::msg::ControlModeReport::msg::XXX
         */
        using AutowareControlMode = autoware_vehicle_msgs::msg::ControlModeReport;

        /**
         * @brief Alias for gear modes in autoware_vehicle_msgs::msg::GearCommand::msg::XXX
         */
        using AutowareGearCommand = autoware_vehicle_msgs::msg::GearCommand;

        /**
         * @brief Alias for gear modes in autoware_vehicle_msgs::msg::GearReport::msg::XXX
         */
        using AutowareGearReport = autoware_vehicle_msgs::msg::GearReport;

        // Constructor & Destructor

        VilmaInterface();
        ~VilmaInterface();

    private:
        // Attributes

        /* Mutexes */
        std::mutex mutex_joystick_command_;
        std::mutex mutex_ma_timer_;
        std::mutex mutex_vilma_state_;
        std::mutex mutex_vilma_sensors_;
        std::mutex mutex_velocity_controller_;

        /* MA */
        microautobox::maudp ma_udp_client;
        std::vector<double> joystick_command_;
        std::vector<double> to_ma_vector_;
        std::vector<double> from_ma_vector_;
        int to_ma_length_;
        int from_ma_length_;

        /* Vehicle Control */
        int ma_operation_mode_;
        std::atomic<uint8_t> vilma_control_mode_;
        std::atomic<double> vilma_steer_tire_angle_;
        std::atomic<double> vilma_steer_tire_speed_;
        std::atomic<double> vilma_longitudinal_speed_;
        std::atomic<bool> change_control_mode_enabled_;
        PIDLMA velocity_controller_;
        double max_steering_tire_angle_rad_;
        double max_gas_value_;
        double max_brake_value_;
        double max_speed_m_s_;
        double brake_user_pressure_set_emergency_;
        bool autonomous_shift_enable_;
        double delay_to_user_command_ms_;
        TempConditionFilter user_command_handler_;
        TempConditionFilter steer_stopped_;
        TempConditionFilter lost_ecu_connection_;

        /* Command validation */
        int autoware_command_time_validity_ms_;
        int communication_timeout_ms_;

        /* Debug */
        bool debug_mode_;
        bool steer_only_mode_;
        double gas_user_value_set_manual_;

        // Methods

        unsigned short to_ma();
        void from_ma(int type_tx, rclcpp::Time stamp);

        bool set_control_mode(uint8_t control_mode);

        double get_steering_value(double steering_tire_angle_rad);

        int get_operation_state(int operation_state_value);

        /// ROS 2

        // Timers

        rclcpp::Time ma_timer_last_stamp_;
        rclcpp::TimerBase::SharedPtr ma_timer_;
        rclcpp::TimerBase::SharedPtr ma_sleep_timer_;
        rclcpp::TimerBase::SharedPtr control_timer_;

        int ma_timer_period_ms_;
        int ma_sleep_period_min_;
        int control_timer_period_ms_;

        void ma_timer_callback();
        void ma_sleep_callback();
        void control_timer_callback();

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

        // HMI

        /* Publishers */

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hmi_throttle_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hmi_braking_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hmi_status_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr hmi_beep_pub_;

        void hmi_beep(const u_int8_t beep_option);
    };

} // namespace vilma
#endif // vilma_interface__HPP_
