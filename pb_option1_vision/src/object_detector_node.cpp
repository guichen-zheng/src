#include "pb_option1_vision/object_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>  // 新增
#include <filesystem>  // C++17 文件系统，用于路径操作
#include <map>
#include <string>
#include <memory>

using namespace std::placeholders;
using namespace pb_option1_vision;

class ObjectDetectorNode : public rclcpp::Node
{
public:
  ObjectDetectorNode() : Node("object_detector_node")
  {
    this->declare_parameter<std::string>("model_path", "yolov8n.onnx");
    this->declare_parameter<float>("confidence_threshold", 0.5);
    this->declare_parameter<std::vector<std::string>>("target_classes", {"cup", "apple", "banana"});
    this->declare_parameter<std::string>("image_topic", "/image");
    this->declare_parameter<std::string>("device", "cpu");

    std::string model_path = this->get_parameter("model_path").as_string();
    float conf_thresh = this->get_parameter("confidence_threshold").as_double();
    target_classes_ = this->get_parameter("target_classes").as_string_array();
    std::string image_topic = this->get_parameter("image_topic").as_string();

    // 处理模型路径：如果不是绝对路径，则尝试在包的 models 目录下查找
    if (!std::filesystem::path(model_path).is_absolute()) {
      std::string package_share = ament_index_cpp::get_package_share_directory("pb_option1_vision");
      std::string candidate = package_share + "/models/" + model_path;
      if (std::filesystem::exists(candidate)) {
        model_path = candidate;
        RCLCPP_INFO(this->get_logger(), "使用包内模型: %s", model_path.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "未找到包内模型，尝试当前路径: %s", model_path.c_str());
      }
    }

    RCLCPP_INFO(this->get_logger(), "初始化YOLO检测器，模型：%s", model_path.c_str());
    try {
      detector_ = std::make_unique<YOLODetector>(model_path, conf_thresh);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "加载模型失败: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // 建立目标类别ID映射
    const auto& all_names = detector_->getClassNames();
    for (const auto& target : target_classes_) {
      for (size_t id = 0; id < all_names.size(); ++id) {
        if (all_names[id].find(target) != std::string::npos) {
          target_class_ids_[target] = id;
          RCLCPP_INFO(this->get_logger(), "目标 '%s' 映射到ID %zu (%s)", 
                      target.c_str(), id, all_names[id].c_str());
          break;
        }
      }
    }

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::QoS(10),
      std::bind(&ObjectDetectorNode::image_callback, this, _1));

    object_pub_ = this->create_publisher<std_msgs::msg::String>("/vision/detected_object", 10);
    position_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/vision/object_position", 10);
    annotated_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/annotated_image", 10);

    RCLCPP_INFO(this->get_logger(), "物体检测节点已启动");
    RCLCPP_INFO(this->get_logger(), "这是修改后的版本，包含调试日志");
    RCLCPP_INFO(this->get_logger(), "订阅的图像话题: %s", image_topic.c_str());
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "收到图像，尺寸: %d x %d", msg->width, msg->height);  // 新增

  try {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge转换失败: %s", e.what());
      return;
    }

    auto detections = detector_->detect(cv_ptr->image);
    RCLCPP_INFO(this->get_logger(), "检测到 %zu 个物体", detections.size());  // 新增

    std::string best_target;
    float best_conf = 0.0f;
    geometry_msgs::msg::Point best_position;

    for (const auto& det : detections) {
      for (const auto& pair : target_class_ids_) {
        if (det.class_id == pair.second && det.confidence > best_conf) {
          best_conf = det.confidence;
          best_target = pair.first;
          best_position.x = det.center.x;
          best_position.y = det.center.y;
          best_position.z = det.confidence;
          break;
        }
      }
    }

    if (!best_target.empty()) {
      std_msgs::msg::String obj_msg;
      obj_msg.data = best_target;
      object_pub_->publish(obj_msg);
      position_pub_->publish(best_position);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "检测到: %s, 置信度: %.2f", best_target.c_str(), best_conf);
    }

    // 绘制标注图像
    RCLCPP_INFO(this->get_logger(), "开始绘制标注图像");  // 新增
    cv::Mat annotated = cv_ptr->image.clone();
    for (const auto& det : detections) {
      cv::rectangle(annotated, det.bbox, cv::Scalar(0, 255, 0), 2);
      std::string label = detector_->getClassNames()[det.class_id] + " " + 
                          cv::format("%.2f", det.confidence);
      cv::putText(annotated, label, det.bbox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
    }
    if (!best_target.empty()) {
      cv::putText(annotated, "Detected: " + best_target, cv::Point(10,30),
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,0), 2);
    }

    RCLCPP_INFO(this->get_logger(), "准备发布标注图像");  // 新增
    auto annotated_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated).toImageMsg();
    annotated_pub_->publish(*annotated_msg);
    RCLCPP_INFO(this->get_logger(), "已发布标注图像");  // 新增

  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV异常: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "标准异常: %s", e.what());
  }
}
  std::unique_ptr<YOLODetector> detector_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr object_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;

  std::vector<std::string> target_classes_;
  std::map<std::string, int> target_class_ids_;  // 目标名称到模型类别ID的映射
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}