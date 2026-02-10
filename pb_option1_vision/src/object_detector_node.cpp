#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>  // 新增

class ObjectDetector : public rclcpp::Node {
public:
  ObjectDetector() : Node("object_detector") {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&ObjectDetector::imageCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("/detected_objects", 10);

    // 动态获取路径
    std::string pkg_share = ament_index_cpp::get_package_share_directory("pb_option1_vision");
    std::string config_path = pkg_share + "/config/detector_params.yaml";
    YAML::Node config = YAML::LoadFile(config_path);
    for (const auto& kv : config["hsv_ranges"]) {
      std::string obj = kv.first.as<std::string>();
      auto range = kv.second;
      cv::Scalar lower(range["lower"][0].as<int>(), range["lower"][1].as<int>(), range["lower"][2].as<int>());
      cv::Scalar upper(range["upper"][0].as<int>(), range["upper"][1].as<int>(), range["upper"][2].as<int>());
      hsv_ranges_[obj] = std::make_pair(lower, upper);
    }
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");  // 改成 ConstPtr
    cv::Mat hsv;
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

    vision_msgs::msg::Detection2DArray detections;
    detections.header = msg->header;

    for (const auto& [obj, range] : hsv_ranges_) {
      cv::Mat mask;
      cv::inRange(hsv, range.first, range.second, mask);
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      if (!contours.empty()) {
        auto largest = *std::max_element(contours.begin(), contours.end(),
        [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
        cv::Rect bbox = cv::boundingRect(largest);

        vision_msgs::msg::Detection2D det;
        det.header = msg->header;
        det.bbox.center.position.x = bbox.x + bbox.width / 2.0;
        det.bbox.center.position.y = bbox.y + bbox.height / 2.0;
        det.bbox.size_x = bbox.width;
        det.bbox.size_y = bbox.height;
        det.results.resize(1);  // 确保 results 有空间
        det.results[0].hypothesis.class_id = obj;
        detections.detections.push_back(det);
      }
    }

    pub_->publish(detections);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;
  std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> hsv_ranges_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetector>());
  rclcpp::shutdown();
  return 0;
}