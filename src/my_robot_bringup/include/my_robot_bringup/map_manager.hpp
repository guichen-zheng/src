#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <string>
#include <vector>
#include <memory>

namespace my_robot_bringup
{

class MapManager
{
public:
    MapManager(rclcpp::Node::SharedPtr node, bool simulation_mode);
    
    void initialize();
    
    // 地图操作
    bool loadMap(const std::string& map_name);
    bool saveMap(const std::string& map_name);
    bool clearMap();
    
    // 地图查询
    std::vector<std::string> getAvailableMaps() const;
    bool mapExists(const std::string& map_name) const;
    nav_msgs::msg::OccupancyGrid getMap() const;
    
    // 地图信息
    std::string getMapPath() const;
    std::string getCurrentMapName() const;
    
private:
    void handleLoadMap(
        const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
        std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);
    
    void handleSaveMap(
        const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
        std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response);
    
    bool parseYamlFile(const std::string& yaml_file,
                      std::string& image_file,
                      double& resolution,
                      double& origin_x,
                      double& origin_y,
                      double& origin_yaw);
    
    bool loadImageFile(const std::string& image_file,
                      nav_msgs::msg::OccupancyGrid& map);
    
    bool saveImageFile(const std::string& image_file,
                      const nav_msgs::msg::OccupancyGrid& map);
    
    // 成员变量
    rclcpp::Node::SharedPtr node_;
    bool simulation_mode_;
    
    std::string map_path_;
    std::string current_map_name_;
    nav_msgs::msg::OccupancyGrid current_map_;
    
    // ROS2服务
    rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_map_service_;
    rclcpp::Service<nav2_msgs::srv::SaveMap>::SharedPtr save_map_service_;
    
    // 发布器
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

} // namespace my_robot_bringup