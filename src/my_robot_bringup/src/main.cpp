#include "my_robot_bringup/navigation_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<my_robot_bringup::NavigationCore>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Navigation Core");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
