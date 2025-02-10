
#include "vilma_interface/vilma_interface.hpp"

namespace vilma
{
    VilmaInterface::VilmaInterface() : Node("vilma_interface")
    {
        // Parameters

        this->declare_parameter("max_steer_tire_angle", 0.44104);
        this->declare_parameter("brake_deadband", -0.1);
        this->declare_parameter("joystick_ma_time_validity_ms", 1000);
        this->declare_parameter("control_cmd_timeout_ms", 1000);
        this->declare_parameter("vilma_ma_msg_timeout_ms", 1000);
        this->declare_parameter("lifecycle_monitor_period_ms", 1000);

        max_steer_tire_angle_ = this->get_parameter("max_steer_tire_angle").as_double();
        brake_deadband_ = this->get_parameter("brake_deadband").as_double();
        joystick_ma_time_validity_ms_ = this->get_parameter("joystick_ma_time_validity_ms").as_int();
        vilma_ma_msg_timeout_ms_ = this->get_parameter("vilma_ma_msg_timeout_ms").as_int();
        control_cmd_timeout_ms_ = this->get_parameter("control_cmd_timeout_ms").as_int();
        lifecycle_monitor_period_ms_ = this->get_parameter("lifecycle_monitor_period_ms").as_int();

        // Timers

        lifecycle_monitor_timer_ = this->create_wall_timer(std::chrono::milliseconds(lifecycle_monitor_period_ms_),
                                                           std::bind(&VilmaInterface::lifecycle_monitor_callback, this));

        // Autoware

        /* Subscribers */

        using std::placeholders::_1;
        using std::placeholders::_2;

        control_cmd_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 1, std::bind(&VilmaInterface::control_cmd_callback, this, _1));

        gear_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
            "/control/command/gear_cmd", 1, std::bind(&VilmaInterface::gear_cmd_callback, this, _1));

        /* Publishers */

        control_mode_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 1);

        gear_report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 1);

        steering_report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 1);

        velocity_report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 1);

        control_mode_msg_.mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
        control_mode_pub_->publish(control_mode_msg_);

        /* Services */

        control_mode_server_ = this->create_service<autoware_vehicle_msgs::srv::ControlModeCommand>(
            "/vehicle/status/control_mode", std::bind(&VilmaInterface::control_mode_cmd_callback, this, _1, _2));

        // VILMA

        /* Subscribers */

        state_ma_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/vilma_ma_ros/state_ma", 1,
                                                                                    std::bind(&VilmaInterface::state_ma_callback, this, std::placeholders::_1));

        sensors_ma_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/vilma_ma_ros/sensors_ma", 1,
                                                                                      std::bind(&VilmaInterface::sensors_ma_callback, this, std::placeholders::_1));

        /* Publishers */

        joystick_ma_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/vilma_ma_ros/joystick_ma", 1);

        /* Messages */

        joystick_ma_msg_data_.reserve(static_cast<int>(JoystickMA::JOYSTICK_MA_DATA_LENGTH));
        std::fill(joystick_ma_msg_data_.begin(), joystick_ma_msg_data_.end(), 0.0);

        joystick_ma_msg_data_[static_cast<int>(JoystickMA::ROS_TIME)] = this->get_clock()->now().seconds();
        joystick_ma_msg_data_[static_cast<int>(JoystickMA::TIME_VALIDITY)] = joystick_ma_time_validity_ms_;

        state_ma_last_stamp_ = 0.0;
        sensors_ma_last_stamp_ = 0.0;
        control_cmd_last_stamp_ = 0.0;
        gear_cmd_last_stamp_ = 0.0;
    }

    void VilmaInterface::lifecycle_monitor_callback()
    {
        /**
         * TODO: Check last time from VILMA topic subscribers messages
         * TODO: Check last time from Autoware control_command topic message
         *
         * TODO: Implement a fail-safe
         *
         */
    }

    void VilmaInterface::control_cmd_callback(const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
    {
        //* Saving topic message as class attribute for velocity controller method.
        control_cmd_msg_ = *msg;

        //* Casting from steer tire angle in rad to normalized steer position interval [-1, 1]
        joystick_ma_msg_data_[static_cast<int>(JoystickMA::STEER_VALUE)] = msg->lateral.steering_tire_angle / max_steer_tire_angle_;

        //* Updating last time stamp for this topic
        control_cmd_last_stamp_ = static_cast<double>(msg->stamp.sec) + static_cast<double>(msg->stamp.nanosec) * 1e-9;
    }

    void VilmaInterface::gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
    {
        //* Updating last time stamp for this topic
        gear_cmd_last_stamp_ = static_cast<double>(msg->stamp.sec) + static_cast<double>(msg->stamp.nanosec) * 1e-9;

        //* Selecting gear state from Autoware command
        switch (msg->command)
        {
        case autoware_vehicle_msgs::msg::GearCommand::NEUTRAL:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<double>(JoystickMA::GEAR_COMMAND_NEUTRAL);
            break;
        case autoware_vehicle_msgs::msg::GearCommand::REVERSE:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<double>(JoystickMA::GEAR_COMMAND_REVERSE);
            break;
        case autoware_vehicle_msgs::msg::GearCommand::DRIVE:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<double>(JoystickMA::GEAR_COMMAND_DRIVE);
            break;
        case autoware_vehicle_msgs::msg::GearCommand::NONE:
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<double>(JoystickMA::GEAR_COMMAND_OFF);
            break;

        default:
            break;
        }
    }

    void VilmaInterface::control_mode_cmd_callback(
        const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
        const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
    {
        //* Request switching control mode from Autoware command:
        response->success = switch_control_mode(request->mode);
    }

    void VilmaInterface::state_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
    {
        //* Updating last time stamp for this topic
        state_ma_last_stamp_ = msg->data[static_cast<int>(StateMA::ROS_TIME)];

        std_msgs::msg::Header header;

        header.frame_id = std::string("base_link");
        header.stamp = this->get_clock()->now();

        /* Steering report publisher */
        {
            autoware_vehicle_msgs::msg::SteeringReport steering_report_msg_;
            steering_report_msg_.stamp = header.stamp;
            steering_report_msg_.steering_tire_angle = msg->data[static_cast<int>(StateMA::STEER_TIRE_ANGLE)];
            steering_report_pub_->publish(steering_report_msg_);
        }

        /* Velocity report publisher */
        {
            autoware_vehicle_msgs::msg::VelocityReport velocity_report_msg_;
            velocity_report_msg_.header = header;
            velocity_report_msg_.lateral_velocity = msg->data[static_cast<int>(StateMA::LATERAL_VELOCITY)];
            velocity_report_msg_.longitudinal_velocity = msg->data[static_cast<int>(StateMA::LONGITUDINAL_SPEED)];
            velocity_report_msg_.heading_rate = msg->data[static_cast<int>(StateMA::ANGULAR_YAW_SPEED)];
            velocity_report_pub_->publish(velocity_report_msg_);
        }

        /* Vehicle velocity control */
        {
            if (control_mode_msg_.mode == control_mode_msg_.AUTONOMOUS && control_mode_msg_.mode == control_mode_msg_.AUTONOMOUS_VELOCITY_ONLY)
            {

                //* Check if control_cmd_msg_ message is valid
                if (this->get_clock()->now().seconds() - control_cmd_last_stamp_ > control_cmd_timeout_ms_)
                {
                    //* If not, change vehicle to manual mode
                    // TODO: Set a emergency mode in future...
                    switch_control_mode(autoware_vehicle_msgs::srv::ControlModeCommand::Request::MANUAL); // ! Maybe use this time of data is odd...
                }
                else
                {
                    //* Control vehicle longitudinal velocity
                    control_vilma_velocity(msg->data[static_cast<int>(StateMA::LONGITUDINAL_SPEED)]);
                }
            }

            joystick_ma_msg_.data = joystick_ma_msg_data_;
            joystick_ma_pub_->publish(joystick_ma_msg_);
        }

        /* Control mode report publisher */
        {
            control_mode_msg_.stamp = header.stamp;
            control_mode_pub_->publish(control_mode_msg_);
        }
    }

    void VilmaInterface::sensors_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
    {

        //* Updating last time stamp for this topic
        sensors_ma_last_stamp_ = msg->data[static_cast<int>(SensorsMA::ROS_TIME)];

        builtin_interfaces::msg::Time stamp = this->get_clock()->now();

        /* Gear report publisher */
        {
            autoware_vehicle_msgs::msg::GearReport gear_report_msg_;

            gear_report_msg_.stamp = stamp;

            switch (static_cast<int>(msg->data[static_cast<int>(SensorsMA::GEAR_STATE)]))
            {
            case static_cast<int>(SensorsMA::GEAR_OFF):
                gear_report_msg_.report = autoware_vehicle_msgs::msg::GearCommand::NEUTRAL;
                break;

            case static_cast<int>(SensorsMA::GEAR_N):
                gear_report_msg_.report = autoware_vehicle_msgs::msg::GearCommand::NONE;
                break;

            case static_cast<int>(SensorsMA::GEAR_R):
                gear_report_msg_.report = autoware_vehicle_msgs::msg::GearCommand::REVERSE;
                break;

            case static_cast<int>(SensorsMA::GEAR_D):
                gear_report_msg_.report = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
                break;

            default:
                break;
            }

            gear_report_pub_->publish(gear_report_msg_);
        }

        // TODO: Check brake pressure
        {
        }

        // TODO: Check EMERGENCY BUTTON
        {
        }

        // TODO: Check steering wheel user torque
        {
        }

        // TODO: CHECK Operational state, GEAR, STEER, GAS and BRAKE commands and status, if applicable, change control mode.
        {
        }
    }

    void VilmaInterface::control_vilma_velocity(double longitudinal_velocity)
    {
        double control_action, velocity_error, control_dt;

        joystick_ma_msg_data_[static_cast<int>(JoystickMA::ROS_TIME)] = this->get_clock()->now().seconds();

        control_dt = joystick_ma_msg_data_[static_cast<int>(JoystickMA::ROS_TIME)] - joystick_ma_msg_.data[static_cast<int>(JoystickMA::ROS_TIME)];

        velocity_error = control_cmd_msg_.longitudinal.velocity - longitudinal_velocity;

        control_action = velocity_controller_.calculate(velocity_error, control_dt);

        if (control_action <= brake_deadband_) // Brake control action
        {
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::BRAKE_VALUE)] = -control_action;
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::BRAKE_COMMAND)] = static_cast<double>(JoystickMA::BRAKE_COMMAND_AUTO);
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GAS_VALUE)] = 0.0;
        }
        else if (control_action >= 0) // Gas control action
        {
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GAS_VALUE)] = control_action;
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::BRAKE_VALUE)] = 0.0;
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::BRAKE_COMMAND)] = static_cast<double>(JoystickMA::BRAKE_COMMAND_OFF);
        }
        else // Engine braking
        {
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GAS_VALUE)] = 0.0;
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::BRAKE_VALUE)] = 0.0;
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::BRAKE_COMMAND)] = static_cast<double>(JoystickMA::BRAKE_COMMAND_OFF);
        }
    }

    /**
     *   ! Maybe before assign the response as success, is important to wait the response from VILMA_MA_ROS
     *
     */
    bool VilmaInterface::switch_control_mode(int control_mode)
    {
        //* Swithing control mode:
        switch (control_mode)
        {
        case autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS:
            //* Change to fully autonomous mode:

            if (autonomous_available())
            {
                //* Resetting controller
                velocity_controller_.reset();

                //* Setting gas pedal command in position mode [0.0, 1.0]
                joystick_ma_msg_data_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<double>(JoystickMA::GAS_COMMAND_POSITION);

                //* Setting steering control in position mode [-1.0, 1.0]
                joystick_ma_msg_data_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<double>(JoystickMA::STEER_COMMAND_POSITION);

                //* Updating control mode report with new control mode
                control_mode_msg_.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;

                //* Service response informing the control mode change was successful
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Fail to switch control mode to autonomous. Autonomous mode unavailable.");
                return false;
            }

            break;

        case autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS_STEER_ONLY:
            //* Change to only autonomous steering control mode:

            if (autonomous_available())
            {
                //* Setting gas pedal in manual mode
                joystick_ma_msg_data_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<double>(JoystickMA::GAS_COMMAND_OFF);

                //* Setting steering control in position mode [-1.0, 1.0]
                joystick_ma_msg_data_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<double>(JoystickMA::STEER_COMMAND_POSITION);

                //* Updating control mode report with new control mode
                control_mode_msg_.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_STEER_ONLY;

                //* Service response informing the control mode change was successful
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Fail to switch control mode to autonomous. Autonomous mode unavailable.");
                return false;
            }
            break;

        case autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS_VELOCITY_ONLY:
            //* Change to only autonomous velocity control mode:

            if (autonomous_available())
            {
                //* Resetting controller
                velocity_controller_.reset();

                //* Setting gas pedal command in position mode [0.0, 1.0]
                joystick_ma_msg_data_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<double>(JoystickMA::GAS_COMMAND_POSITION);

                //* Setting steering control in manual mode
                joystick_ma_msg_data_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<double>(JoystickMA::STEER_COMMAND_OFF);

                //* Updating control mode report with new control mode
                control_mode_msg_.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;

                //* Service response informing the control mode change was successful
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Fail to switch control mode to autonomous. Autonomous mode unavailable.");
                return false;
            }

            break;

        case autoware_vehicle_msgs::srv::ControlModeCommand::Request::MANUAL:

            //* Setting gas pedal in manual mode
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<double>(JoystickMA::GAS_COMMAND_OFF);

            //* Setting steering control in manual mode
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<double>(JoystickMA::STEER_COMMAND_OFF);

            //* Setting gear changing in manual mode
            joystick_ma_msg_data_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<double>(JoystickMA::GEAR_COMMAND_OFF);

            //* Updating control mode report with new control mode
            control_mode_msg_.mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;

            //* Service response informing the control mode change was successful
            return true;
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), "Fail to switch control mode. Unsupported mode.");
            return false;
            break;
        }

        return false;
    }

    bool VilmaInterface::autonomous_available()
    {
        // * Firstly, autonomous will be available when vilma_ma topics are in valid lifetime.
        // TODO: Interesting conditions: vehicle stopped,

        double current_time = this->get_clock()->now().seconds();

        return ((current_time - sensors_ma_last_stamp_ <= vilma_ma_msg_timeout_ms_ / 1000.0) &&
                (current_time - state_ma_last_stamp_ <= vilma_ma_msg_timeout_ms_ / 1000.0));
    }

} // namespace vilma
