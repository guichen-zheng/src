#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <mutex>

namespace my_robot_bringup
{

class SlamProcessor : public rclcpp::Node
{
public:
    explicit SlamProcessor(bool simulation_mode = false);
    
    void initialize();
    
    void processScan(const sensor_msgs::msg::LaserScan& scan);
    void updatePose(const nav_msgs::msg::Odometry& odom);
    
    nav_msgs::msg::OccupancyGrid getMap() const;
    geometry_msgs::msg::PoseStamped getPose() const;
    
    bool saveMap(const std::string& filename);
    bool loadMap(const std::string& filename);
    
private:
    void setupParameters();
    void setupPublishers();
    void setupSubscribers();
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    void publishMap();
    void publishPose();
    
    bool simulation_mode_;
    bool initialized_;
    
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PoseStamped current_pose_;
    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::mutex map_mutex_;
    std::mutex pose_mutex_;
};

} // namespace my_robot_bringup