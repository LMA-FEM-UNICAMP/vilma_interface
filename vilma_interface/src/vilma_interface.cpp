
#include "vilma_interface/vilma_interface.hpp"

namespace vilma
{
    VilmaInterface::VilmaInterface() : Node("vilma_interface")
    {

        // Placeholders        
        using std::placeholders::_1;
        using std::placeholders::_2;

        // Parameters

        int pc_udp_port;
        int ma_udp_port;
        int udp_timeout_ms;
        std::string ma_ip;

        autonomous_shift_enable_ = false;

        ma_operation_mode_ = static_cast<int>(OperationModeMA::MANUAL_MODE);
        vilma_control_mode_ = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;

        to_ma_vector_.reserve(to_ma_length_);
        to_ma_vector_.resize(to_ma_length_, 0.0);
        from_ma_vector_.reserve(from_ma_length_);
        from_ma_vector_.resize(from_ma_length_, 0.0);

        sensors_ma_msg.data.reserve(from_ma_length_ + 1);
        sensors_ma_msg.data.resize(from_ma_length_ + 1, 0.0);
        state_ma_msg.data.reserve(from_ma_length_ + 1);
        state_ma_msg.data.resize(from_ma_length_ + 1, 0.0);

        if (!ma_udp_client.configure(pc_udp_port, ma_udp_port, ma_ip, boost::posix_time::millisec(udp_timeout_ms), to_ma_length_, from_ma_length_))
        {
            RCLCPP_ERROR(this->get_logger(), "UDP port of PC is not accessible");
        }

        // ROS 2

        rclcpp::SubscriptionOptions autoware_sub_options;
        rclcpp::SubscriptionOptions debug_sub_options;

        timers_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        subscribers_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        autoware_sub_options.callback_group = subscribers_callback_group;

        debug_sub_options.callback_group = subscribers_callback_group;

        ma_timer_ = this->create_wall_timer(std::chrono::minutes(2), std::bind(&VilmaInterface::ma_timer_callback, this),
                                            timers_callback_group_);

        ma_sleep_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&VilmaInterface::ma_sleep_callback, this),
                                            timers_callback_group_);

        ma_sleep_->cancel();

        control_cmd_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", rclcpp::QoS{1}, std::bind(&VilmaInterface::control_cmd_callback, this, _1),
            autoware_sub_options);

        gear_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
            "/control/command/gear_cmd", rclcpp::QoS{1}, std::bind(&VilmaInterface::gear_cmd_callback, this, _1),
            autoware_sub_options);

        engage_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::Engage>(
            "/vehicle/engage", rclcpp::QoS{1}, std::bind(&VilmaInterface::engage_callback, this, _1), autoware_sub_options);

        control_mode_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
            "/vehicle/status/control_mode", rclcpp::QoS{1});

        gear_report_pub_ =
            this->create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", rclcpp::QoS{1});

        steering_report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
            "/vehicle/status/steering_status", rclcpp::QoS{1});

        velocity_report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", rclcpp::QoS{1});

        control_mode_request_server_ = this->create_service<autoware_vehicle_msgs::srv::ControlModeCommand>(
            "/control/control_mode_request", std::bind(&VilmaInterface::control_mode_request_callback, this, _1, _2));

        joystick_ma_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/vilma_ma_debug/joystick_ma", rclcpp::QoS{1}, std::bind(&VilmaInterface::joystick_ma_callback, this, _1), debug_sub_options);

        state_ma_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>("/vilma_ma_debug/state_ma", rclcpp::QoS{1});

        sensors_ma_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>("/vilma_ma_debug/sensors_ma", rclcpp::QoS{1});
    }

    VilmaInterface::~VilmaInterface()
    {
        ma_udp_client.close_udp_socket();
    }

    unsigned short VilmaInterface::to_ma()
    {
        unsigned short rx_type;

        mutex_to_ma_vector_.lock();
        to_ma_msg_vector = to_ma_vector_;
        mutex_to_ma_vector_.unlock();

        if (to_ma_msg_vector[0] == 0.0)
        {
            rx_type = static_cast<int>(RxTypeMA::ONLY_RECEIVE_DATA);
            to_ma_msg_vector[0] = ma_operation_mode_;
            to_ma_msg_vector[1] = this->get_clock()->now().seconds();
            to_ma_msg_vector[2] = 152;

            std::fill(to_ma_msg_vector.begin() + 3, to_ma_msg_vector.end(), 0.0);
        }

        else
        {
            rx_type = static_cast<int>(RxTypeMA::JOYSTICK_MODE_COMMAND);
        }

        return rx_type;
    }

    void VilmaInterface::from_ma(int type_tx, rclcpp::Time stamp)
    {

        //* Assembling header for messages
        std_msgs::msg::Header header_msg;
        header_msg.stamp = stamp;
        header_msg.frame_id = std::string("base_link");

        switch (type_tx)
        {
        case static_cast<int>(TxTypeMA::SENSORS_MA):
        {
            //* Publishing debug topic
            sensors_ma_msg.data[0] = stamp.seconds();
            std::copy(from_ma_vector_.begin(), from_ma_vector_.end(), sensors_ma_msg.data.begin() + 1);
            sensors_ma_pub_->publish(sensors_ma_msg);

            //* Publish gear status topic
            autoware_vehicle_msgs::msg::GearReport gear_report_msg;
            gear_report_msg.stamp = header_msg.stamp;

            switch (static_cast<int>(sensors_ma_msg.data[static_cast<int>(SensorsMA::GEAR_STATE)]))
            {
            case static_cast<int>(SensorsMA::GEAR_OFF):
                gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::NONE;
                break;

            case static_cast<int>(SensorsMA::GEAR_D):
                gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::DRIVE;
                break;

            case static_cast<int>(SensorsMA::GEAR_R):
                gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::REVERSE;
                break;

            case static_cast<int>(SensorsMA::GEAR_N):
                gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::NEUTRAL;
                break;

            default:
                break;
            }

            gear_report_pub_->publish(gear_report_msg);

            break;
        }

        case static_cast<int>(TxTypeMA::STATE_MA):
        {
            //* Publish debug topic
            state_ma_msg.data[0] = stamp.seconds();
            std::copy(from_ma_vector_.begin(), from_ma_vector_.end(), state_ma_msg.data.begin() + 1);
            state_ma_pub_->publish(state_ma_msg);

            //* Publish velocity status topic
            autoware_vehicle_msgs::msg::VelocityReport velocity_report_msg;
            velocity_report_msg.header = header_msg;
            velocity_report_msg.heading_rate = state_ma_msg.data[static_cast<int>(StateMA::ANGULAR_YAW_SPEED)];
            velocity_report_msg.lateral_velocity = state_ma_msg.data[static_cast<int>(StateMA::LATERAL_VELOCITY)];
            velocity_report_msg.longitudinal_velocity = state_ma_msg.data[static_cast<int>(StateMA::LONGITUDINAL_SPEED)];
            velocity_report_pub_->publish(velocity_report_msg);

            //* Publish steering status topic
            autoware_vehicle_msgs::msg::SteeringReport steering_report_msg;
            steering_report_msg.stamp = header_msg.stamp;
            steering_report_msg.steering_tire_angle = state_ma_msg.data[static_cast<int>(StateMA::STEER_TIRE_ANGLE)];
            steering_report_pub_->publish(steering_report_msg);

            //* Publish control mode report topic
            autoware_vehicle_msgs::msg::ControlModeReport control_mode_report_msg;
            control_mode_report_msg.mode = vilma_control_mode_;
            control_mode_pub_->publish(control_mode_report_msg);

            break;
        }

        default:
            break;
        }
    }

    bool VilmaInterface::set_control_mode(uint8_t control_mode)
    {
        switch (control_mode)
        {
        case autoware_vehicle_msgs::msg::ControlModeReport::MANUAL:

            mutex_to_ma_vector_.lock();
            to_ma_vector_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<int>(JoystickMA::GAS_COMMAND_OFF);

            to_ma_vector_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<int>(JoystickMA::STEER_COMMAND_OFF);

            to_ma_vector_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<int>(JoystickMA::GEAR_COMMAND_OFF);
            mutex_to_ma_vector_.unlock();

            vilma_control_mode_ = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;

            return true;

            break;

        case autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS:

            mutex_to_ma_vector_.lock();
            to_ma_vector_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<int>(JoystickMA::GAS_COMMAND_POSITION);

            to_ma_vector_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<int>(JoystickMA::STEER_COMMAND_POSITION);
            mutex_to_ma_vector_.unlock();

            vilma_control_mode_ = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;

            return true;

            break;

        case autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_STEER_ONLY:

            mutex_to_ma_vector_.lock();
            to_ma_vector_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<int>(JoystickMA::GAS_COMMAND_OFF);

            to_ma_vector_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<int>(JoystickMA::STEER_COMMAND_POSITION);
            mutex_to_ma_vector_.unlock();

            vilma_control_mode_ = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_STEER_ONLY;

            return true;

            break;

        case autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_VELOCITY_ONLY:

            mutex_to_ma_vector_.lock();
            to_ma_vector_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<int>(JoystickMA::GAS_COMMAND_POSITION);

            to_ma_vector_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<int>(JoystickMA::STEER_COMMAND_OFF);
            mutex_to_ma_vector_.unlock();

            vilma_control_mode_ = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;

            return true;

            break;

        default:
            break;
        }
    }

    void VilmaInterface::ma_timer_callback()
    {
        rclcpp::Time stamp = this->get_clock()->now();

        unsigned short rx_type = to_ma();
        unsigned short tx_type = 32768;

        std::string udp_request_output = ma_udp_client.ma_udp_request(&from_ma_vector_[0], &tx_type, &to_ma_vector_[0], rx_type);

        if (!udp_request_output.empty())
        {
            if (ma_udp_client.get_udp_error() < 12) // Print 10 first errors
            {
                RCLCPP_ERROR(this->get_logger(), "%s", udp_request_output.c_str());
            }
            else if (ma_udp_client.get_udp_error() > 1000) // Sleep UDP for some time and reconnect
            {
                ma_timer_->cancel();
                ma_sleep_->reset();
            }
        }
        else
        {
            from_ma(rx_type, stamp);

            // TODO: MA delay
        }

        to_ma_vector_[0] = 0.0;
    }

    void VilmaInterface::ma_sleep_callback()
    {
        if (ma_udp_client.open_udp_socket())
        {
            RCLCPP_INFO(this->get_logger(), "Reconnection with MA successful.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Error to open socket.");
        }

        ma_sleep_->cancel();
        ma_timer_->reset();
    }

    void VilmaInterface::control_cmd_callback(const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
    {
    }

    void VilmaInterface::gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
    {
        if (autonomous_shift_enable_)
        {
            switch (msg->command)
            {
            case autoware_vehicle_msgs::msg::GearCommand::NEUTRAL:
                mutex_to_ma_vector_.lock();
                to_ma_vector_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<int>(JoystickMA::GEAR_COMMAND_NEUTRAL);
                mutex_to_ma_vector_.unlock();
                break;

            case autoware_vehicle_msgs::msg::GearCommand::REVERSE:
                mutex_to_ma_vector_.lock();
                to_ma_vector_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<int>(JoystickMA::GEAR_COMMAND_REVERSE);
                mutex_to_ma_vector_.unlock();
                break;

            case autoware_vehicle_msgs::msg::GearCommand::DRIVE:
                mutex_to_ma_vector_.lock();
                to_ma_vector_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<int>(JoystickMA::GEAR_COMMAND_DRIVE);
                mutex_to_ma_vector_.unlock();
                break;

            default:
                break;
            }
        }
    }

    void VilmaInterface::engage_callback(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
    {
        set_control_mode(autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS);
    }

    void VilmaInterface::control_mode_request_callback(
        const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
        const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
    {
        response->success = set_control_mode(request->mode);
    }

    void VilmaInterface::joystick_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
    {
    }

} // namespace vilma
