
#include "vilma_interface/vilma_interface.hpp"

namespace vilma
{

    /**
     *
     * @brief Class constructor
     * @param None
     * @return void
     */
    VilmaInterface::VilmaInterface() : Node("vilma_interface")
    {
        /// Placeholders

        using std::placeholders::_1;
        using std::placeholders::_2;

        /// Parameters

        /* ROS2 parameters declaration*/
        this->declare_parameter("ma_timer_period_ms", 10);
        this->declare_parameter("ma_sleep_period_min", 2);
        this->declare_parameter("pc_udp_port", 5001);
        this->declare_parameter("ma_udp_port", 5001);
        this->declare_parameter("udp_timeout_ms", 4);
        this->declare_parameter("ma_ip", "192.168.140.3");
        this->declare_parameter("autonomous_shift_enable", 0);
        this->declare_parameter("kp_vel", 0.0015);
        this->declare_parameter("ki_vel", 0.0);
        this->declare_parameter("kd_vel", 0.0);
        this->declare_parameter("brake_deadband", -0.1);
        this->declare_parameter("max_steering_tire_angle_rad", 25.27 * M_PI / 180.0);
        this->declare_parameter("max_gas_value", 1.0);
        this->declare_parameter("max_brake_value", 1.0);
        this->declare_parameter("max_speed_km_h", 60.0);
        this->declare_parameter("speed_reference_ramp_rate", 3.0);
        this->declare_parameter("brake_user_pressure_emergency", 10.0);
        this->declare_parameter("joystick_command_time_validity_ms", 500);

        /* UDP communication parameters */
        int pc_udp_port = this->get_parameter("pc_udp_port").as_int();
        int ma_udp_port = this->get_parameter("ma_udp_port").as_int();
        int udp_timeout_ms = this->get_parameter("udp_timeout_ms").as_int();
        std::string ma_ip = this->get_parameter("ma_ip").as_string();

        /* PID control parameters*/
        double kp_vel = this->get_parameter("kp_vel").as_double();
        double ki_vel = this->get_parameter("ki_vel").as_double();
        double kd_vel = this->get_parameter("kd_vel").as_double();

        /* MA's control command time validity*/
        double joystick_command_time_validity_ms = this->get_parameter("joystick_command_time_validity_ms").as_int();

        /* Timers period */
        ma_timer_period_ms_ = this->get_parameter("ma_timer_period_ms").as_int();
        ma_sleep_period_min_ = this->get_parameter("ma_sleep_period_min").as_int();

        /* Vehicle parameters */
        brake_deadband_ = this->get_parameter("brake_deadband").as_double();
        max_steering_tire_angle_rad_ = this->get_parameter("max_steering_tire_angle_rad").as_double();
        max_gas_value_ = this->get_parameter("max_gas_value").as_double();
        max_brake_value_ = this->get_parameter("max_brake_value").as_double();
        max_speed_m_s_ = this->get_parameter("max_speed_km_h").as_double() / 3.6;
        speed_reference_ramp_rate_ = this->get_parameter("speed_reference_ramp_rate").as_double();
        autonomous_shift_enable_ = this->get_parameter("autonomous_shift_enable").as_bool();
        brake_user_pressure_set_emergency_ = this->get_parameter("brake_user_pressure_set_emergency").as_double();

        /* Vehicle variables initialization */
        ma_operation_mode_ = static_cast<int>(OperationModeMA::MANUAL_MODE);
        vilma_control_mode_ = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
        change_control_mode_enabled_ = true;

        /* MA messages configuration */
        to_ma_length_ = 10;
        from_ma_length_ = 30;

        joystick_command_.reserve(to_ma_length_);
        joystick_command_.resize(to_ma_length_, 0.0);
        joystick_command_[static_cast<int>(JoystickMA::TIME_VALIDITY)] = joystick_command_time_validity_ms;

        to_ma_vector_.reserve(to_ma_length_);
        to_ma_vector_.resize(to_ma_length_, 0.0);

        from_ma_vector_.reserve(from_ma_length_);
        from_ma_vector_.resize(from_ma_length_, 0.0);

        sensors_ma_msg.data.reserve(from_ma_length_ + 1);
        sensors_ma_msg.data.resize(from_ma_length_ + 1, 0.0);
        state_ma_msg.data.reserve(from_ma_length_ + 1);
        state_ma_msg.data.resize(from_ma_length_ + 1, 0.0);

        /* Configure UDP communication wih MA*/
        if (!ma_udp_client.configure(pc_udp_port, ma_udp_port, ma_ip, boost::posix_time::millisec(udp_timeout_ms), to_ma_length_, from_ma_length_))
        {
            RCLCPP_ERROR(this->get_logger(), "UDP port of PC is not accessible");
        }

        /* Configure velocity controller */
        velocity_controller_.configure(kp_vel, kd_vel, ki_vel, this->get_clock()->now().seconds(), speed_reference_ramp_rate_);

        /// ROS2 entities

        rclcpp::SubscriptionOptions autoware_sub_options;
        rclcpp::SubscriptionOptions debug_sub_options;

        timers_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        subscribers_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        autoware_sub_options.callback_group = subscribers_callback_group;

        debug_sub_options.callback_group = subscribers_callback_group;

        ma_timer_ = this->create_wall_timer(std::chrono::milliseconds(ma_timer_period_ms_), std::bind(&VilmaInterface::ma_timer_callback, this),
                                            timers_callback_group_);

        ma_sleep_ = this->create_wall_timer(std::chrono::minutes(ma_sleep_period_min_), std::bind(&VilmaInterface::ma_sleep_callback, this),
                                            timers_callback_group_);

        ma_sleep_->cancel();

        /* From Autoware */
        control_cmd_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", rclcpp::QoS{1}, std::bind(&VilmaInterface::control_cmd_callback, this, _1),
            autoware_sub_options);

        gear_cmd_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
            "/control/command/gear_cmd", rclcpp::QoS{1}, std::bind(&VilmaInterface::gear_cmd_callback, this, _1),
            autoware_sub_options);

        engage_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::Engage>(
            "/vehicle/engage", rclcpp::QoS{1}, std::bind(&VilmaInterface::engage_callback, this, _1), autoware_sub_options);

        control_mode_request_server_ = this->create_service<autoware_vehicle_msgs::srv::ControlModeCommand>(
            "/control/control_mode_request", std::bind(&VilmaInterface::control_mode_request_callback, this, _1, _2));

        /* To Autoware */
        control_mode_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
            "/vehicle/status/control_mode", rclcpp::QoS{1});

        gear_report_pub_ =
            this->create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", rclcpp::QoS{1});

        steering_report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
            "/vehicle/status/steering_status", rclcpp::QoS{1});

        velocity_report_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", rclcpp::QoS{1});

        /* Debug topics */
        joystick_ma_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/vilma_ma_debug/joystick_ma", rclcpp::QoS{1}, std::bind(&VilmaInterface::joystick_ma_callback, this, _1), debug_sub_options);

        state_ma_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>("/vilma_ma_debug/state_ma", rclcpp::QoS{1});

        sensors_ma_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>("/vilma_ma_debug/sensors_ma", rclcpp::QoS{1});
    }

    /**
     *
     * @brief Class destructor
     * @param
     * @return
     */
    VilmaInterface::~VilmaInterface()
    {
        //* Closing UDP communication with MA.
        ma_udp_client.close_udp_socket();
    }

    /**
     *
     * @brief Build vector with message data from Autoware to MA.
     * @param None
     * @return RX type value of the UDP message
     */
    unsigned short VilmaInterface::to_ma()
    {
        unsigned short rx_type;

        mutex_joystick_command_.lock();
        to_ma_vector_ = joystick_command_;
        mutex_joystick_command_.unlock();

        if (to_ma_vector_[0] == 0.0)
        {
            rx_type = static_cast<int>(RxTypeMA::ONLY_RECEIVE_DATA);
            to_ma_vector_[0] = ma_operation_mode_;
            to_ma_vector_[1] = this->get_clock()->now().seconds();
            to_ma_vector_[2] = 152;

            std::fill(to_ma_vector_.begin() + 3, to_ma_vector_.end(), 0.0);
        }

        else
        {
            rx_type = static_cast<int>(RxTypeMA::JOYSTICK_MODE_COMMAND);
        }

        return rx_type;
    }

    /**
     *
     * @brief Process data received from MA through UDP.
     * @param type_tx type of UDP message
     * @param stamp time when the message was requested
     * @return void
     */
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

            //* Emergency -- User braking
            if (sensors_ma_msg.data[static_cast<int>(SensorsMA::BRAKE_USER_PRESSURE)] >= brake_user_pressure_set_emergency_)
            {
                set_control_mode(autoware_vehicle_msgs::msg::ControlModeReport::MANUAL);
                change_control_mode_enabled_ = false;
            }
            else
            {
                change_control_mode_enabled_ = true;
            }

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

    /**
     *
     * @brief Change vehicle control mode configuring the command values in the joystick message.
     * @param control_mode desired control mode in autoware_vehicle_msgs::msg::ControlModeReport constants format.
     * @return feedback of success or fail in change control mode.
     */
    bool VilmaInterface::set_control_mode(uint8_t control_mode)
    {
        if (change_control_mode_enabled_)
        {

            switch (control_mode)
            {
            case autoware_vehicle_msgs::msg::ControlModeReport::MANUAL:

                mutex_joystick_command_.lock();
                joystick_command_[static_cast<int>(JoystickMA::ROS_TIME)] = this->get_clock()->now().seconds();

                joystick_command_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<int>(JoystickMA::GAS_COMMAND_OFF);

                joystick_command_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<int>(JoystickMA::STEER_COMMAND_OFF);

                joystick_command_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<int>(JoystickMA::GEAR_COMMAND_OFF);
                mutex_joystick_command_.unlock();

                vilma_control_mode_ = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;

                return true;

                break;

            case autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS:

                mutex_joystick_command_.lock();
                joystick_command_[static_cast<int>(JoystickMA::ROS_TIME)] = this->get_clock()->now().seconds();

                joystick_command_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<int>(JoystickMA::GAS_COMMAND_POSITION);

                joystick_command_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<int>(JoystickMA::STEER_COMMAND_POSITION);
                mutex_joystick_command_.unlock();

                vilma_control_mode_ = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;

                return true;

                break;

            case autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_STEER_ONLY:

                mutex_joystick_command_.lock();
                joystick_command_[static_cast<int>(JoystickMA::ROS_TIME)] = this->get_clock()->now().seconds();

                joystick_command_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<int>(JoystickMA::GAS_COMMAND_OFF);

                joystick_command_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<int>(JoystickMA::STEER_COMMAND_POSITION);
                mutex_joystick_command_.unlock();

                vilma_control_mode_ = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_STEER_ONLY;

                return true;

                break;

            case autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_VELOCITY_ONLY:

                mutex_joystick_command_.lock();
                joystick_command_[static_cast<int>(JoystickMA::ROS_TIME)] = this->get_clock()->now().seconds();

                joystick_command_[static_cast<int>(JoystickMA::GAS_COMMAND)] = static_cast<int>(JoystickMA::GAS_COMMAND_POSITION);

                joystick_command_[static_cast<int>(JoystickMA::STEER_COMMAND)] = static_cast<int>(JoystickMA::STEER_COMMAND_OFF);
                mutex_joystick_command_.unlock();

                vilma_control_mode_ = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;

                return true;

                break;

            default:
                break;
            }
        }
        else
        {
            return false;
        }
    }

    /**
     *
     * @brief Timer callback that make the UDP request to MA, sending and receiving data.
     * @param None
     * @return void
     */
    void VilmaInterface::ma_timer_callback()
    {
        rclcpp::Time stamp = this->get_clock()->now();

        rclcpp::Duration ma_timer_dt = stamp - last_stamp_;

        last_stamp_ = stamp;

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

            RCLCPP_INFO(this->get_logger(), "MA period: %f", ma_timer_dt.seconds());
        }

        // If joystick command was the sent one, the stamp is resetted to avoid send the same message again.
        mutex_joystick_command_.lock();
        joystick_command_[0] = (joystick_command_[0] == to_ma_vector_[0]) ? 0.0 : joystick_command_[0];
        mutex_joystick_command_.unlock();
    }

    /**
     *
     * @brief Timer callback for wait some time to reconnect MA UDP communication.
     * @param None
     * @return void
     */
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

    /**
     *
     * @brief Return steering normalized value [-1, 1] for a steering tire angle value;
     * @param steering_tire_angle_rad Steering tire angle in radians
     * @return Normalized value for the desired angle
     */
    double VilmaInterface::get_steering_value(double steering_tire_angle_rad)
    {
        return steering_tire_angle_rad / max_steering_tire_angle_rad_;
    }

    /**
     *
     * @brief Receive the control reference for lateral and longitudinal control and process the steering, gas and brake values,
     *        applying low-level controller to longitudinal velocity.
     * @param msg Ackermann control topic message
     * @return void
     */
    void VilmaInterface::control_cmd_callback(const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
    {
        double gas_value = 0.0;
        double brake_value = 0.0;
        double brake_command = static_cast<double>(JoystickMA::BRAKE_COMMAND_OFF);

        double control_action = velocity_controller_.calculate(state_ma_msg.data[static_cast<int>(StateMA::LONGITUDINAL_SPEED)],
                                                               msg->longitudinal.velocity, this->get_clock()->now().seconds());

        // Negative control action above engine brake limit
        if (control_action <= brake_deadband_)
        {
            // Assign the control action as braking percentage
            brake_value = (-control_action + brake_deadband_) / (1.0 - brake_deadband_);

            // Setting brake mode in autonomous
            brake_command = static_cast<double>(JoystickMA::BRAKE_COMMAND_AUTO);
        }
        // Positive control action
        else if (control_action >= 0)
        {
            // Assign control action as gas pedal position
            gas_value = control_action;
        }

        mutex_joystick_command_.lock();
        joystick_command_[static_cast<int>(JoystickMA::ROS_TIME)] = this->get_clock()->now().seconds();

        joystick_command_[static_cast<int>(JoystickMA::STEER_VALUE)] = get_steering_value(msg->lateral.steering_tire_angle);

        joystick_command_[static_cast<int>(JoystickMA::GAS_VALUE)] = get_steering_value(msg->lateral.steering_tire_angle);

        joystick_command_[static_cast<int>(JoystickMA::BRAKE_COMMAND)] = brake_command;
        joystick_command_[static_cast<int>(JoystickMA::BRAKE_VALUE)] = brake_value;
        mutex_joystick_command_.unlock();
    }

    /**
     *
     * @brief Recieve gear command and process the request in joystick command vector.
     * @param msg gear command received by topic.
     * @return void
     */
    void VilmaInterface::gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
    {
        if (autonomous_shift_enable_)
        {
            switch (msg->command)
            {
            case autoware_vehicle_msgs::msg::GearCommand::NEUTRAL:
                mutex_joystick_command_.lock();
                joystick_command_[static_cast<int>(JoystickMA::ROS_TIME)] = this->get_clock()->now().seconds();

                joystick_command_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<int>(JoystickMA::GEAR_COMMAND_NEUTRAL);
                mutex_joystick_command_.unlock();
                break;

            case autoware_vehicle_msgs::msg::GearCommand::REVERSE:
                mutex_joystick_command_.lock();
                joystick_command_[static_cast<int>(JoystickMA::ROS_TIME)] = this->get_clock()->now().seconds();

                joystick_command_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<int>(JoystickMA::GEAR_COMMAND_REVERSE);
                mutex_joystick_command_.unlock();
                break;

            case autoware_vehicle_msgs::msg::GearCommand::DRIVE:
                mutex_joystick_command_.lock();
                joystick_command_[static_cast<int>(JoystickMA::ROS_TIME)] = this->get_clock()->now().seconds();

                joystick_command_[static_cast<int>(JoystickMA::GEAR_STATE)] = static_cast<int>(JoystickMA::GEAR_COMMAND_DRIVE);
                mutex_joystick_command_.unlock();
                break;

            default:
                break;
            }
        }
    }

    /**
     *
     * @brief Recieve engage topic, switching control mode to manual ou auto.
     * @param msg Engage topic message.
     * @return void
     */
    void VilmaInterface::engage_callback(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
    {
        if (msg->engage)
        {
            set_control_mode(autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS);
        }
        else
        {
            set_control_mode(autoware_vehicle_msgs::msg::ControlModeReport::MANUAL);
        }
    }

    /**
     *
     * @brief Change control mode service server.
     * @param request required control mode.
     * @param response feedback of success in change control mode.
     * @return void
     */
    void VilmaInterface::control_mode_request_callback(
        const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
        const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
    {
        response->success = set_control_mode(request->mode);
    }

    /**
     *
     * @brief Recieve joystick command topic and process to be sended to MA.
     * @param msg JoystickMA message
     * @return void
     */
    void VilmaInterface::joystick_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
    {
        mutex_joystick_command_.lock();
        joystick_command_ = msg->data;
        mutex_joystick_command_.unlock();
    }

} // namespace vilma
