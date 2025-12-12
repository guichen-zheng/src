#include "my_robot_bringup/gazebo_manager.hpp"
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace my_robot_bringup
{

GazeboManager::GazeboManager()
    : Node("gazebo_manager")
{
    spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    
    RCLCPP_INFO(get_logger(), "Gazebo Manager initialized");
}

bool GazeboManager::spawnRobot(const std::string& name, 
                              const geometry_msgs::msg::Pose& pose,
                              const std::string& model,
                              const std::string& color,
                              const std::string& namespace_)
{
    // 等待服务
    if (!spawn_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(get_logger(), "Spawn service not available");
        return false;
    }
    
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    
    request->name = name;
    request->robot_namespace = namespace_;
    request->initial_pose = pose;
    
    // 获取机器人描述文件
    std::string robot_xml = getRobotDescription(model, color);
    if (robot_xml.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to get robot description for model: %s", model.c_str());
        return false;
    }
    
    request->xml = robot_xml;
    
    auto future = spawn_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(shared_from_this(), future) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(get_logger(), "Successfully spawned robot: %s", name.c_str());
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to spawn robot: %s", response->status_message.c_str());
            return false;
        }
    }
    
    RCLCPP_ERROR(get_logger(), "Service call failed");
    return false;
}

bool GazeboManager::deleteRobot(const std::string& name)
{
    if (!delete_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(get_logger(), "Delete service not available");
        return false;
    }
    
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = name;
    
    auto future = delete_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(shared_from_this(), future) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(get_logger(), "Successfully deleted robot: %s", name.c_str());
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to delete robot: %s", name.c_str());
            return false;
        }
    }
    
    return false;
}

bool GazeboManager::resetWorld()
{
    // 实现世界重置逻辑
    RCLCPP_INFO(get_logger(), "Reset world not implemented yet");
    return false;
}

std::string GazeboManager::getRobotDescription(const std::string& model, const std::string& color)
{
    // 这里应该从my_robot_description包中加载机器人模型
    // 简化实现：返回一个基本的SDF描述
    
    std::string robot_sdf = R"(
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name=')" + model + R"('>
    <pose>0 0 0.1 0 0 0</pose>
    <link name='base_link'>
      <pose>0 0 0.1 0 0 0</pose>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>0.2</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>0.2</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>)" + (color == "red" ? "1.0 0.0 0.0" : "0.0 0.0 1.0") + R"( 1</ambient>
          <diffuse>)" + (color == "red" ? "1.0 0.0 0.0" : "0.0 0.0 1.0") + R"( 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>)";
    
    return robot_sdf;
}

} // namespace my_robot_bringup