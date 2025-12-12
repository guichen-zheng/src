// src/map_manager.cpp
#include "my_robot_bringup/map_manager.hpp"

namespace my_robot_bringup
{

MapManager::MapManager(rclcpp::Node::SharedPtr node, bool simulation_mode)
    : node_(node), simulation_mode_(simulation_mode)
{
    initialize();
}

void MapManager::initialize()
{
    // 基础初始化
    map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map_manager/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    
    RCLCPP_INFO(node_->get_logger(), "Map Manager initialized");
}

bool MapManager::loadMap(const std::string& map_name)
{
    RCLCPP_INFO(node_->get_logger(), "Loading map: %s", map_name.c_str());
    return true;
}

bool MapManager::saveMap(const std::string& map_name)
{
    RCLCPP_INFO(node_->get_logger(), "Saving map: %s", map_name.c_str());
    return true;
}

nav_msgs::msg::OccupancyGrid MapManager::getMap() const
{
    return current_map_;
}

} // namespace my_robot_bringup