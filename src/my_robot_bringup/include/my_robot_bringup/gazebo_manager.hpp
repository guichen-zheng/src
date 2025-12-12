#pragma once

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <memory>

namespace my_robot_bringup
{

class GazeboManager : public rclcpp::Node
{
public:
    GazeboManager();
    
    bool spawnRobot(const std::string& name, 
                   const geometry_msgs::msg::Pose& pose,
                   const std::string& model = "standard_robot",
                   const std::string& color = "red",
                   const std::string& namespace_ = "");
    
    bool deleteRobot(const std::string& name);
    
    bool resetWorld();
    
private:
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
    
    std::string getRobotDescription(const std::string& model, const std::string& color);
};

} // namespace my_robot_bringup