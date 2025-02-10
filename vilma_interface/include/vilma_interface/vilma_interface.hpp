
#ifndef vilma_interface__HPP_
#define vilma_interface__HPP_

// std includes

#include <algorithm>
#include <functional>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>

// ROS includes

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/header.hpp"

/* Autoware includes */

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>

// Personal libraries includes

#include "vilma_interface/vilma_ma_labeling.hpp"
#include "vilma_interface/PIDLMA.hpp"

namespace vilma
{
    class VilmaInterface : public rclcpp::Node
    {
    public:
    
        VilmaInterface();

    private:

        // Attributes

        PIDLMA velocity_controller_;

        double state_ma_last_stamp_;
        double sensors_ma_last_stamp_;
        double control_cmd_last_stamp_;
        double gear_cmd_last_stamp_;

        // Methods

        void control_vilma_velocity(double longitudinal_velocity);
        bool switch_control_mode(int control_mode);
        bool autonomous_available();


        // -- ROS2 elements --

        // Parameters

        double max_steer_tire_angle_;
        double brake_deadband_;
        int joystick_ma_time_validity_ms_;
        int vilma_ma_msg_timeout_ms_;
        int control_cmd_timeout_ms_;
        int lifecycle_monitor_period_ms_;

        // Timers

        rclcpp::TimerBase::SharedPtr lifecycle_monitor_timer_;

        // Callbacks

        void lifecycle_monitor_callback();

        // Autoware

        /* Subscribers */

        rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
        rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;

        /* Publishers */

        rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_pub_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_pub_;

        /* Services */

        rclcpp::Service<autoware_vehicle_msgs::srv::ControlModeCommand>::SharedPtr control_mode_server_;

        /* Messages */

        autoware_vehicle_msgs::msg::ControlModeReport control_mode_msg_;

        autoware_control_msgs::msg::Control control_cmd_msg_;

        /* Callbacks */

        void control_cmd_callback(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
        void gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
        void control_mode_cmd_callback(const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
                                       const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response);

        // VILMA

        /* Subscribers */

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_ma_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensors_ma_sub_;

        /* Publishers */

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_ma_pub_;

        /* Messages */

        std_msgs::msg::Float64MultiArray joystick_ma_msg_;
        std::vector<double> joystick_ma_msg_data_;

        /* Callbacks */

        void state_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
        void sensors_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
    };

} // namespace vilma
#endif // vilma_interface__HPP_
