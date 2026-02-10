#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

class CommandInterpreter : public rclcpp::Node {
public:
  CommandInterpreter() : Node("command_interpreter") {
    sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detected_objects", 10, std::bind(&CommandInterpreter::detectionsCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);  // 或专用话题
  }

private:
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
    if (msg->detections.empty()) return;

    std::string obj_class = msg->detections[0].results[0].hypothesis.class_id;
    geometry_msgs::msg::Twist cmd;

    if (obj_class == "apple") {  // 前进
      cmd.linear.x = 0.5;
    } else if (obj_class == "cup") {  // 左转
      cmd.angular.z = 0.5;
    } else if (obj_class == "banana") {  // 右转
      cmd.angular.z = -0.5;
    }

    pub_->publish(cmd);
  }

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandInterpreter>());
  rclcpp::shutdown();
  return 0;
}