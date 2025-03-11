
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
#include "vilma_interface/PIDLMA.hpp"
#include "vilma_interface/maudp.h"

// UDP libraries

#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace vilma
{
    class VilmaInterface : public rclcpp::Node
    {

    public:
        VilmaInterface();
        ~VilmaInterface();

    private:
        // Parameters

        bool autonomous_shift_enable_;

        // Attributes

        std::vector<double> to_ma_vector_;
        std::vector<double> to_ma_msg_vector;
        std::vector<double> from_ma_vector_;
        int ma_operation_mode_;
        int to_ma_length_;
        int from_ma_length_;

        microautobox::maudp ma_udp_client;

        std::mutex mutex_to_ma_vector_;

        uint8_t vilma_control_mode_;

        // Methods

        unsigned short to_ma();

        void from_ma(int type_tx, rclcpp::Time stamp);

        bool set_control_mode(uint8_t control_mode);

        // ROS 2

        // Timers

        rclcpp::TimerBase::SharedPtr ma_timer_;
        rclcpp::TimerBase::SharedPtr ma_sleep_;

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

        rclcpp::CallbackGroup::SharedPtr timers_callback_group_;
        rclcpp::CallbackGroup::SharedPtr subscribers_callback_group;

        /* Callbacks */

        void control_cmd_callback(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
        void gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
        void engage_callback(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
        void control_mode_request_callback(const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
                                           const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response);

        /* Messages */

        // VILMA

        /* Subscribers */

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joystick_ma_sub_;

        /* Publishers */

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_ma_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sensors_ma_pub_;

        /* Messages */
        std_msgs::msg::Float64MultiArray state_ma_msg;
        std_msgs::msg::Float64MultiArray sensors_ma_msg;

        /* Callbacks */

        void joystick_ma_callback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);
    };

} // namespace vilma
#endif // vilma_interface__HPP_
