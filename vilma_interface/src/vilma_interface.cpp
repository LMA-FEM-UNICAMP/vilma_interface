
#include "vilma_interface/vilma_interface.hpp"

namespace vilma
{
    VilmaInterface::VilmaInterface() : Node("vilma_interface")
    {

        // Subscribers

        using std::placeholders::_1;
        using std::placeholders::_2;

        control_cmd_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
            "/control/command/control_cmd", 1, std::bind(&VilmaInterface::control_cmd_callback, this, _1));

        gear_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
            "/control/command/gear_cmd", 1, std::bind(&VilmaInterface::gear_cmd_callback, this, _1));

        // Publishers

        control_mode_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 1);

        gear_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 1);

        steering_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 1);

        velocity_report_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 1);

        // Services

        control_mode_server_ = this->create_service<autoware_auto_vehicle_msgs::srv::ControlModeCommand>(
            "/vehicle/status/control_mode", std::bind(&VilmaInterface::control_mode_cmd_callback, this, _1, _2));

        // VILMA

        /* Subscribers */

        state_ma_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/vilma_ma_ros/state_ma", 1,
                                                                                    std::bind(&VilmaInterface::state_ma_callback, this, std::placeholders::_1));

        sensors_ma_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/vilma_ma_ros/sensors_ma", 1,
                                                                                      std::bind(&VilmaInterface::sensors_ma_callback, this, std::placeholders::_1));

        /* Publishers */

        joystick_ma_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/vilma_ma_ros/joystick_ma", 1);
    }

    void VilmaInterface::control_cmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr)
    {
    }

    void VilmaInterface::gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr)
    {
    }

    void VilmaInterface::control_mode_cmd_callback(
        const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
        const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
    {
    }

    void VilmaInterface::state_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
    {
    }

    void VilmaInterface::sensors_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr)
    {
    }

} // namespace vilma
