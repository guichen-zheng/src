#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <string>

using namespace std::placeholders;

class CommandInterpreterNode : public rclcpp::Node
{
public:
  CommandInterpreterNode() : Node("command_interpreter_node")
  {
    // 物体到动作的映射 (根据新要求：苹果右转、香蕉左转、杯子右转)
    object_action_map_ = {
    {"cup",    {"forward", 0.3, 0.0}},   // 杯子 -> 前行
    {"apple",  {"right",   0.0, -0.5}},  // 苹果 -> 右转
    {"banana", {"left",    0.0, 0.5}}    // 香蕉 -> 左转
    };
    // 订阅检测到的物体
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/vision/detected_object", 10,
      std::bind(&CommandInterpreterNode::object_callback, this, _1));

    // 发布速度命令
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    action_pub_ = this->create_publisher<std_msgs::msg::String>("/vision/current_action", 10);

    // 定时器用于执行动作（持续发送速度）
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&CommandInterpreterNode::timer_callback, this));

    // 超时停止
    last_detection_time_ = this->now();
    stop_timeout_ = 2.0; // 2秒

    RCLCPP_INFO(this->get_logger(), "命令解释节点已启动，映射规则：");
    for (const auto& [obj, info] : object_action_map_) {
      RCLCPP_INFO(this->get_logger(), "  %s -> %s (linear=%.1f, angular=%.1f)",
                  obj.c_str(), info.action.c_str(), info.linear_x, info.angular_z);
    }
  }

private:
  struct ActionInfo {
    std::string action;
    double linear_x;
    double angular_z;
  };

  void object_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto it = object_action_map_.find(msg->data);
    if (it != object_action_map_.end()) {
      last_detection_time_ = this->now();
      current_action_ = it->second;

      RCLCPP_INFO(this->get_logger(), "检测到 %s -> 执行 %s", 
                  msg->data.c_str(), current_action_.action.c_str());

      std_msgs::msg::String action_msg;
      action_msg.data = msg->data + ":" + current_action_.action;
      action_pub_->publish(action_msg);
    }
  }

  void timer_callback()
  {
    auto time_since_detection = (this->now() - last_detection_time_).seconds();
    if (time_since_detection > stop_timeout_) {
      if (current_action_.linear_x != 0.0 || current_action_.angular_z != 0.0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "未检测到物体，停止运动");
        current_action_ = ActionInfo{}; // 清零
      }
      // 发布零速度
      geometry_msgs::msg::Twist twist;
      cmd_vel_pub_->publish(twist);
      return;
    }

    if (current_action_.linear_x != 0.0 || current_action_.angular_z != 0.0) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = current_action_.linear_x;
      twist.angular.z = current_action_.angular_z;
      cmd_vel_pub_->publish(twist);
    }
  }

  std::map<std::string, ActionInfo> object_action_map_;
  ActionInfo current_action_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_detection_time_;
  double stop_timeout_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandInterpreterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}