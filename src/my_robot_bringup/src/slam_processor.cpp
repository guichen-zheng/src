#include "my_robot_bringup/slam_processor.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

namespace my_robot_bringup
{

SlamProcessor::SlamProcessor(bool simulation_mode)
    : Node("slam_processor"),
      simulation_mode_(simulation_mode),
      is_initialized_(false),
      has_initial_pose_(false)
{
    RCLCPP_INFO(get_logger(), "Initializing SLAM Processor");
    initialize();
}

void SlamProcessor::initialize()
{
    initParameters();
    setupCommunications();
    
    // 初始化TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // 初始化地图
    current_map_.header.frame_id = map_frame_;
    current_map_.info.resolution = map_resolution_;
    current_map_.info.width = map_width_;
    current_map_.info.height = map_height_;
    current_map_.info.origin.position.x = map_origin_x_;
    current_map_.info.origin.position.y = map_origin_y_;
    current_map_.info.origin.orientation.w = 1.0;
    
    // 初始化为未知区域
    current_map_.data.resize(map_width_ * map_height_, -1);
    
    // 处理定时器
    processing_timer_ = create_wall_timer(
        100ms,  // 10Hz
        [this]() {
            process();
        });
    
    RCLCPP_INFO(get_logger(), "SLAM Processor initialized");
}

void SlamProcessor::initParameters()
{
    // 声明参数
    declare_parameter("laser_topic", "/scan");
    declare_parameter("odom_topic", "/odom");
    declare_parameter("map_frame", "map");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    
    // 地图参数
    declare_parameter("map_resolution", 0.05);
    declare_parameter("map_width", 400);
    declare_parameter("map_height", 400);
    declare_parameter("map_origin_x", -10.0);
    declare_parameter("map_origin_y", -10.0);
    
    // SLAM算法参数
    declare_parameter("scan_matching_threshold", 0.1);
    declare_parameter("loop_closure_threshold", 1.0);
    declare_parameter("max_iterations", 100);
    
    // 模式特定参数
    if (simulation_mode_) {
        declare_parameter("simulation_scan_noise", 0.01);
        declare_parameter("simulation_odom_noise", 0.02);
    } else {
        declare_parameter("real_scan_filter_size", 3);
        declare_parameter("real_odom_covariance_factor", 1.5);
    }
    
    // 获取参数值
    laser_topic_ = get_parameter("laser_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    
    map_resolution_ = get_parameter("map_resolution").as_double();
    map_width_ = get_parameter("map_width").as_int();
    map_height_ = get_parameter("map_height").as_int();
    map_origin_x_ = get_parameter("map_origin_x").as_double();
    map_origin_y_ = get_parameter("map_origin_y").as_double();
    
    scan_matching_threshold_ = get_parameter("scan_matching_threshold").as_double();
    loop_closure_threshold_ = get_parameter("loop_closure_threshold").as_double();
    max_iterations_ = get_parameter("max_iterations").as_int();
}

void SlamProcessor::setupCommunications()
{
    // 发布器
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/slam_pose", 10);
    
    // 订阅器
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SlamProcessor::laserCallback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&SlamProcessor::odomCallback, this, std::placeholders::_1));
    
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        std::bind(&SlamProcessor::initialPoseCallback, this, std::placeholders::_1));
}

void SlamProcessor::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    scan_queue_.push(*msg);
}

void SlamProcessor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!has_initial_pose_) {
        // 如果没有初始位姿，使用第一个里程计数据作为初始位姿
        current_pose_ = msg->pose.pose;
        last_odom_pose_ = current_pose_;
        has_initial_pose_ = true;
        is_initialized_ = true;
        RCLCPP_INFO(get_logger(), "SLAM initialized with odometry");
    } else {
        last_odom_pose_ = msg->pose.pose;
    }
}

void SlamProcessor::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    current_pose_ = msg->pose.pose;
    last_odom_pose_ = current_pose_;
    has_initial_pose_ = true;
    is_initialized_ = true;
    RCLCPP_INFO(get_logger(), "SLAM initialized with manual pose");
}

void SlamProcessor::process()
{
    if (!is_initialized_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 处理所有积压的激光数据
    while (!scan_queue_.empty()) {
        auto scan = scan_queue_.front();
        scan_queue_.pop();
        
        processScan(scan);
    }
    
    // 更新地图
    updateMap();
    
    // 发布当前位姿
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = current_pose_;
    pose_pub_->publish(pose_msg);
    
    // 发布TF变换
    publishTransform();
    
    // 定期发布地图
    static auto last_map_publish = now();
    auto now_time = now();
    if ((now_time - last_map_publish).seconds() > 1.0) {
        current_map_.header.stamp = now_time;
        map_pub_->publish(current_map_);
        last_map_publish = now_time;
    }
}

void SlamProcessor::processScan(const sensor_msgs::msg::LaserScan& scan)
{
    // 简单的扫描匹配实现
    // 在实际应用中，这里应该实现ICP或类似算法
    
    // 计算里程计变化
    auto pose_change = computePoseChange(last_odom_pose_, current_pose_);
    
    // 应用扫描匹配来修正位姿
    // 这里只是一个简单示例
    double dx = pose_change.position.x;
    double dy = pose_change.position.y;
    double dyaw = 2.0 * asin(pose_change.orientation.z);
    
    // 简单的障碍物检测和地图更新
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];
        if (std::isinf(range) || std::isnan(range) || range > scan.range_max || range < scan.range_min) {
            continue;
        }
        
        // 计算激光点在机器人坐标系中的位置
        float angle = scan.angle_min + i * scan.angle_increment;
        double x_laser = range * cos(angle);
        double y_laser = range * sin(angle);
        
        // 转换到地图坐标系
        double x_map = current_pose_.position.x + 
                      cos(current_pose_.orientation.z) * x_laser -
                      sin(current_pose_.orientation.z) * y_laser;
        double y_map = current_pose_.position.y +
                      sin(current_pose_.orientation.z) * x_laser +
                      cos(current_pose_.orientation.z) * y_laser;
        
        // 转换到地图网格坐标
        int grid_x = static_cast<int>((x_map - map_origin_x_) / map_resolution_);
        int grid_y = static_cast<int>((y_map - map_origin_y_) / map_resolution_);
        
        if (grid_x >= 0 && grid_x < map_width_ && grid_y >= 0 && grid_y < map_height_) {
            int index = grid_y * map_width_ + grid_x;
            current_map_.data[index] = 100;  // 障碍物
        }
    }
}

void SlamProcessor::updateMap()
{
    // 这里可以实现更复杂的地图更新逻辑
    // 比如概率更新、空闲区域标记等
}

void SlamProcessor::publishTransform()
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    
    transform.transform.translation.x = current_pose_.position.x;
    transform.transform.translation.y = current_pose_.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = current_pose_.orientation;
    
    tf_broadcaster_->sendTransform(transform);
}

nav_msgs::msg::OccupancyGrid SlamProcessor::getCurrentMap() const
{
    return current_map_;
}

geometry_msgs::msg::Pose SlamProcessor::getCurrentPose() const
{
    return current_pose_;
}

bool SlamProcessor::saveMap(const std::string& filename)
{
    RCLCPP_INFO(get_logger(), "Saving map to: %s", filename.c_str());
    // 实际实现应该将地图保存为PGM和YAML文件
    return true;
}

bool SlamProcessor::loadMap(const std::string& filename)
{
    RCLCPP_INFO(get_logger(), "Loading map from: %s", filename.c_str());
    // 实际实现应该从文件加载地图
    return true;
}

geometry_msgs::msg::Pose SlamProcessor::computePoseChange(
    const geometry_msgs::msg::Pose& pose1,
    const geometry_msgs::msg::Pose& pose2)
{
    geometry_msgs::msg::Pose change;
    
    // 计算位置变化
    change.position.x = pose2.position.x - pose1.position.x;
    change.position.y = pose2.position.y - pose1.position.y;
    change.position.z = 0.0;
    
    // 计算角度变化
    double yaw1 = 2.0 * atan2(pose1.orientation.z, pose1.orientation.w);
    double yaw2 = 2.0 * atan2(pose2.orientation.z, pose2.orientation.w);
    double dyaw = yaw2 - yaw1;
    
    // 归一化角度
    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
    
    change.orientation.z = sin(dyaw / 2.0);
    change.orientation.w = cos(dyaw / 2.0);
    
    return change;
}

} // namespace my_robot_bringup