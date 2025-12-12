#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace my_robot_bringup
{

class NavigationCore : public rclcpp::Node
{
public:
    NavigationCore();
    
private:
    void setupParameters();
    void setupSubscribers();
    void setupPublishers();
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    void publishVelocity(double linear_x, double angular_z);
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    
    double max_linear_speed_;
    double max_angular_speed_;
};

}
