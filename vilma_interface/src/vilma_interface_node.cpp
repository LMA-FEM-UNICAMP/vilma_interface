
#include "vilma_interface/vilma_interface.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vilma::VilmaInterface>());
    rclcpp::shutdown();

    return 0;
}
