#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class CommandInterpreter : public rclcpp::Node {
public:
  CommandInterpreter() : Node("command_interpreter") {
    sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detected_objects", 10, std::bind(&CommandInterpreter::detectionsCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 加载 YAML 动作映射
    std::string pkg_share = ament_index_cpp::get_package_share_directory("pb_option1_vision");
    std::string config_path = pkg_share + "/config/detector_params.yaml";
    YAML::Node config = YAML::LoadFile(config_path);

    action_map_["apple"] = config["action_mapping"]["apple"].as<std::string>();
    action_map_["banana"] = config["action_mapping"]["banana"].as<std::string>();
  }

private:
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    if (msg->detections.empty()) return;

    std::string class_id = msg->detections[0].results[0].hypothesis.class_id;

    geometry_msgs::msg::Twist cmd;
    if (class_id == "apple" && action_map_["apple"] == "right_turn") {
      cmd.angular.z = -0.5;  // 右转
    } else if (class_id == "banana" && action_map_["banana"] == "left_turn") {
      cmd.angular.z = 0.5;  // 左转
    } else {
      return;  // 无有效物体，不发布
    }

    pub_->publish(cmd);
  }

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

  std::map<std::string, std::string> action_map_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandInterpreter>());
  rclcpp::shutdown();
  return 0;
}