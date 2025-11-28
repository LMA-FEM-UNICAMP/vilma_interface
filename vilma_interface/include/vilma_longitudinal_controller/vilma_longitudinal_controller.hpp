
#include <rclcpp/rclcpp.hpp>

class VilmaLongitudinalController
{
private:
    rclcpp::Node interface_node;

    rclcpp::Publisher<vilma_interface::msg::PIDLMADebug>::SharedPtr pid_control_debug_pub_;

    
public:
    VilmaLongitudinalController();
    ~VilmaLongitudinalController();
};
