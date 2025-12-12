#include "my_robot_bringup/navigation_core.hpp"

using namespace std::chrono_literals;

namespace my_robot_bringup
{

NavigationCore::NavigationCore() : Node("navigation_core")
{
    RCLCPP_INFO(get_logger(), "Initializing Navigation Core");
    
    setupParameters();
    setupSubscribers();
    setupPublishers();
    
    RCLCPP_INFO(get_logger(), "Navigation Core ready");
}

void NavigationCore::setupParameters()
{
    declare_parameter("max_linear_speed", 0.5);
    declare_parameter("max_angular_speed", 1.0);
    
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    
    RCLCPP_INFO(get_logger(), "Parameters: linear=%.2f, angular=%.2f", 
                max_linear_speed_, max_angular_speed_);
}

void NavigationCore::setupSubscribers()
{
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            RCLCPP_INFO(get_logger(), "Received goal at (%.2f, %.2f)", 
                        msg->pose.position.x, msg->pose.position.y);
            
            // 简单前进
            publishVelocity(0.1, 0.0);
        });
    
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            // 简单避障
            if (!msg->ranges.empty() && msg->ranges[0] < 1.0) {
                RCLCPP_WARN(get_logger(), "Obstacle detected! Stopping.");
                publishVelocity(0.0, 0.0);
            }
        });
}

void NavigationCore::setupPublishers()
{
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void NavigationCore::publishVelocity(double linear_x, double angular_z)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    cmd_vel_pub_->publish(msg);
}

}
