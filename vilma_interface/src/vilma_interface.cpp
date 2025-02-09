
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

        control_mode_msg_.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
        control_mode_pub_->publish(control_mode_msg_);

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

        joystick_ma_msg_data_.reserve(static_cast<int>(JoystickMA::JOYSTICK_MA_DATA_LENGTH), 0.0);

        joystick_ma_msg_data_[static_cast<int>(JoystickMA::TIME_VALIDITY)] = 1000; // could be a parameter
    }

    void VilmaInterface::control_cmd_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
    {
        control_cmd_msg_ = *msg;
    }

    void VilmaInterface::gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
    {
        switch (msg->command)
        {
        case autoware_auto_vehicle_msgs::msg::GearCommand::NEUTRAL:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<double>(JoystickMA::GEAR_COMMAND_NEUTRAL);
            break;
        case autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<double>(JoystickMA::GEAR_COMMAND_REVERSE);
            break;
        case autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<double>(JoystickMA::GEAR_COMMAND_DRIVE);
            break;
        case autoware_auto_vehicle_msgs::msg::GearCommand::NONE:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<double>(JoystickMA::GEAR_COMMAND_OFF);
            break;
        
        default:
            break;
        }
    }

    void VilmaInterface::control_mode_cmd_callback(
        const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
        const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
    {
        switch (request->mode)
        {
        case autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::BRAKE_COMMAND)] = static_cast<double>(JoystickMA::BRAKE_COMMAND_AUTO);
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<double>(JoystickMA::GAS_COMMAND_POSITION);
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<double>(JoystickMA::STEER_COMMAND_POSITION);
            control_mode_msg_.mode = control_mode_msg_.AUTONOMOUS;
            break;

        case autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS_STEER_ONLY:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::BRAKE_COMMAND)] = static_cast<double>(JoystickMA::BRAKE_COMMAND_OFF);
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<double>(JoystickMA::GAS_COMMAND_OFF);
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<double>(JoystickMA::STEER_COMMAND_POSITION);
            control_mode_msg_.mode = control_mode_msg_.AUTONOMOUS_STEER_ONLY;
            break;

        case autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS_VELOCITY_ONLY:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::BRAKE_COMMAND)] = static_cast<double>(JoystickMA::BRAKE_COMMAND_AUTO);
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<double>(JoystickMA::GAS_COMMAND_POSITION);
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<double>(JoystickMA::STEER_COMMAND_OFF);
            control_mode_msg_.mode = control_mode_msg_.AUTONOMOUS_VELOCITY_ONLY;
            break;

        case autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::MANUAL:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::BRAKE_COMMAND)] = static_cast<double>(JoystickMA::BRAKE_COMMAND_OFF);
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<double>(JoystickMA::GAS_COMMAND_OFF);
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<double>(JoystickMA::STEER_COMMAND_OFF);
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<double>(JoystickMA::GEAR_COMMAND_OFF);
            control_mode_msg_.mode = control_mode_msg_.MANUAL;
            break;

        case autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::NO_COMMAND:
            break;

        default:
            break;
        }
    }

    void VilmaInterface::state_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
    {        
        autoware_auto_vehicle_msgs::msg::SteeringReport steering_report_msg_;
        autoware_auto_vehicle_msgs::msg::VelocityReport velocity_report_msg_;

        steering_report_msg_.stamp = this->get_clock()->now().seconds();
        steering_report_msg_.steering_tire_angle = msg->data[static_cast<int>(StateMA::STEER_TIRE_ANGLE)];
        steering_report_pub_->publish(steering_report_msg_);

        velocity_report_msg_.header.stamp = this->get_clock()->now().seconds();
        velocity_report_msg_.header.frame_id = std::string("base_link");
        velocity_report_msg_.lateral_velocity = msg->data[static_cast<int>(StateMA::LATERAL_VELOCITY];
        velocity_report_msg_.longitudinal_velocity = msg->data[static_cast<int>(StateMA::LONGITUDINAL_SPEED];
        velocity_report_msg_.heading_rate = msg->data[static_cast<int>(StateMA::ANGULAR_YAW_SPEED];
        velocity_report_pub_->publish(velocity_report_msg_);

        control_mode_pub_->publish(control_mode_msg_);

        // Velocity control...
        
            // control_cmd_msg_ ...

        joystick_ma_msg_data_[static_cast<int>(JoystickMA::ROS_TIME)] = this->get_clock()->now().seconds();
    }

    void VilmaInterface::sensors_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
    {
        autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg_;

        gear_report_msg_.stamp = this->get_clock()->now().seconds();
        gear_report_msg_.report = msg->data[static_cast<int>(SensorsMA::GEAR_STATE)];
        gear_report_pub_->publish(gear_report_msg_);


    }

} // namespace vilma
