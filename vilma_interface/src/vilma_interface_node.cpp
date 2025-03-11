
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "vilma_interface/vilma_interface.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto vilma_interface_node = std::make_shared<vilma::VilmaInterface>();

    rclcpp::executors::MultiThreadedExecutor mt_executor;
    mt_executor.add_node(vilma_interface_node);

    RCLCPP_INFO(vilma_interface_node->get_logger(), "Starting VILMA's vehicle interface through UDP");
    mt_executor.spin();
    RCLCPP_INFO(vilma_interface_node->get_logger(), "Shutting down VILMA's vehicle interface. \n");

    rclcpp::shutdown();
    return 0;
}
