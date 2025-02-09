
#ifndef vilma_interface__HPP_
#define vilma_interface__HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/srv/control_mode_command.hpp>

#include <algorithm>
#include <functional>
#include <cmath>
#include <cstdlib>
#include <memory>

namespace vilma
{
    class VilmaInterface : public rclcpp::Node
    {
    public:
        VilmaInterface();

    private:
    
        double longitudinal_velocity_; // m/s
        double lateral_velocity_;      // m/s
        double heading_rate_;          // rad/s
        double steer_tire_angle_;      // rad/s

        // Autoware

        /* Subscribers */

        rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_sub_;
        rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;

        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_pub_;

        /* Publishers */

        rclcpp::Service<autoware_auto_vehicle_msgs::srv::ControlModeCommand>::SharedPtr control_mode_server_;

        /* Callbacks */

        void control_cmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr);
        void gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr);
        void control_mode_cmd_callback(const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
                                       const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response);
        // VILMA

        /* Subscribers */

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_ma_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensors_ma_sub_;

        /* Publishers */

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_ma_pub_;

        /* Callbacks */

        void state_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
        void sensors_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
    };

} // namespace vilma
#endif // vilma_interface__HPP_
