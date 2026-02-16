#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

using namespace std::placeholders;

class FollowBehaviorNode : public rclcpp::Node
{
public:
  FollowBehaviorNode() : Node("follow_behavior_node")
  {
    // 声明参数
    this->declare_parameter<bool>("enable_follow", false);
    this->declare_parameter<std::string>("target_object", "cup");
    this->declare_parameter<double>("linear_speed", 0.3);
    this->declare_parameter<double>("angular_speed", 0.5);
    this->declare_parameter<double>("target_area_ratio", 0.3);
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<int>("center_tolerance", 50);

    // 获取参数
    enable_follow_ = this->get_parameter("enable_follow").as_bool();
    target_object_ = this->get_parameter("target_object").as_string();
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();
    target_area_ratio_ = this->get_parameter("target_area_ratio").as_double();
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();
    center_tolerance_ = this->get_parameter("center_tolerance").as_int();

    // 订阅物体类型和位置
    object_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/vision/detected_object", 10,
      std::bind(&FollowBehaviorNode::object_callback, this, _1));
    position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/vision/object_position", 10,
      std::bind(&FollowBehaviorNode::position_callback, this, _1));

    // 发布速度
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 控制定时器
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&FollowBehaviorNode::control_callback, this));

    // 超时检测
    last_update_time_ = this->now();
    timeout_ = 1.0; // 1秒

    RCLCPP_INFO(this->get_logger(), "跟随节点已启动，目标物体: %s", target_object_.c_str());
  }

private:
  void object_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    current_object_ = msg->data;
    last_update_time_ = this->now();
  }

  void position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    object_position_ = *msg;
    last_update_time_ = this->now();
  }

  void control_callback()
  {
    if (!enable_follow_) return;

    // 超时检查
    auto time_since = (this->now() - last_update_time_).seconds();
    if (time_since > timeout_) {
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      return;
    }

    // 检查是否是目标物体
    if (current_object_ != target_object_) return;
    if (!object_position_) return;

    // 计算控制命令
    geometry_msgs::msg::Twist cmd;

    // 图像中心
    double image_center_x = image_width_ / 2.0;
    double error_x = object_position_->x - image_center_x;

    // 角速度控制（对齐）
    if (std::abs(error_x) > center_tolerance_) {
      // 物体在左 => 左转 (正角速度)，但坐标系：图像左为小x，右为大x
      // 为使物体居中，若物体在左（error_x<0），需左转（正角速度），即 angular_z = +K * (-error_x)
      cmd.angular.z = angular_speed_ * (error_x / image_center_x); // 正比例
      // 限制范围
      cmd.angular.z = std::max(-angular_speed_, std::min(angular_speed_, cmd.angular.z));
    }

    // 线速度控制（前进），简单策略：只要检测到且置信度足够，就前进
    double confidence = object_position_->z;
    if (confidence > 0.5) {
      cmd.linear.x = linear_speed_ * 0.8; // 保持一定速度
    }

    cmd_vel_pub_->publish(cmd);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "跟随控制: linear=%.2f, angular=%.2f", cmd.linear.x, cmd.angular.z);
  }

  bool enable_follow_;
  std::string target_object_;
  double linear_speed_;
  double angular_speed_;
  double target_area_ratio_;
  int image_width_;
  int image_height_;
  int center_tolerance_;

  std::string current_object_;
  std::optional<geometry_msgs::msg::Point> object_position_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr object_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  rclcpp::Time last_update_time_;
  double timeout_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowBehaviorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}