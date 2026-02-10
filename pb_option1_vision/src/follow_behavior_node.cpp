#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>  // 新增

class FollowBehavior : public rclcpp::Node {
public:
  FollowBehavior() : Node("follow_behavior") {
    sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detected_objects", 10, std::bind(&FollowBehavior::detectionsCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 动态获取路径
    std::string pkg_share = ament_index_cpp::get_package_share_directory("pb_option1_vision");
    std::string config_path = pkg_share + "/config/follow_params.yaml";
    YAML::Node config = YAML::LoadFile(config_path);
    target_distance_ = config["target_distance"].as<double>(1.0);
    linear_gain_ = config["linear_gain"].as<double>(0.5);
    angular_gain_ = config["angular_gain"].as<double>(0.003);
    image_width_ = config["image_width"].as<int>(640);
  }

private:
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    if (msg->detections.empty()) return;

    auto& det = msg->detections[0];
    double cx = det.bbox.center.position.x - image_width_ / 2.0;
    double distance = estimateDistance(det.bbox.size_x, det.bbox.size_y);

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = -cx * angular_gain_;

    if (std::abs(cx) < 40) {
      if (distance > target_distance_ + 0.15) {
        cmd.linear.x = linear_gain_ * (distance - target_distance_);
      } else if (distance < target_distance_ - 0.15) {
        cmd.linear.x = -linear_gain_ * (target_distance_ - distance);
      }
    }

    pub_->publish(cmd);
  }

  double estimateDistance(double width, double height) {
    return 1000.0 / (width * height);  // 示例估算
  }

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  double target_distance_, linear_gain_, angular_gain_;
  int image_width_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowBehavior>());
  rclcpp::shutdown();
  return 0;
}