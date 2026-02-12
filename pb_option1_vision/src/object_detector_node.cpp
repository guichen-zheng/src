#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

class ObjectDetector : public rclcpp::Node {
public:
  ObjectDetector() : Node("object_detector") {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image", 10, std::bind(&ObjectDetector::imageCallback, this, std::placeholders::_1));

    pub_detections_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("/detected_objects", 10);
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detection_markers", 10);
    
    // 新增：发布带框和文字的图像（推荐在RViz Image面板中显示这个）
    pub_annotated_ = this->create_publisher<sensor_msgs::msg::Image>("/detection/image_annotated", 10);

    // 加载参数
    std::string pkg_share = ament_index_cpp::get_package_share_directory("pb_option1_vision");
    std::string config_path = pkg_share + "/config/detector_params.yaml";
    YAML::Node config = YAML::LoadFile(config_path);

    auto lower = config["hsv_range"]["lower"];
    auto upper = config["hsv_range"]["upper"];
    hsv_lower_ = cv::Scalar(lower["h"].as<int>(), lower["s"].as<int>(), lower["v"].as<int>());
    hsv_upper_ = cv::Scalar(upper["h"].as<int>(), upper["s"].as<int>(), upper["v"].as<int>());

    kernel_size_ = config["morphology"]["kernel_size"].as<int>(5);
    min_area_ = config["detection"]["min_area"].as<double>(600.0);

    apple_min_circ_ = config["apple"]["min_circularity"].as<double>(0.62);
    apple_aspect_min_ = config["apple"]["aspect_ratio_min"].as<double>(0.7);
    apple_aspect_max_ = config["apple"]["aspect_ratio_max"].as<double>(1.45);
    apple_min_area_ = config["apple"]["min_area"].as<double>(800.0);

    banana_max_circ_ = config["banana"]["max_circularity"].as<double>(0.40);
    banana_aspect_min_ = config["banana"]["aspect_ratio_min"].as<double>(1.9);
    banana_aspect_max_ = config["banana"]["aspect_ratio_max"].as<double>(6.0);
    banana_min_area_ = config["banana"]["min_area"].as<double>(600.0);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat image = cv_ptr->image.clone();  // 用于画框
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, hsv_lower_, hsv_upper_, mask);

    // 形态学处理
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size_, kernel_size_));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1,-1), 2);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    vision_msgs::msg::Detection2DArray detections;
    detections.header = msg->header;
    visualization_msgs::msg::MarkerArray marker_array;

    for (const auto& contour : contours) {
      double area = cv::contourArea(contour);
      if (area < min_area_) continue;

      double perimeter = cv::arcLength(contour, true);
      double circularity = (perimeter > 0) ? (4 * CV_PI * area / (perimeter * perimeter)) : 0.0;

      cv::Rect bbox = cv::boundingRect(contour);
      double aspect = static_cast<double>(bbox.width) / bbox.height;

      // 计算该区域平均色调（Hue）来区分苹果和香蕉
      cv::Mat roi_mask = mask(bbox);
      cv::Scalar mean_hsv = cv::mean(hsv(bbox), roi_mask);
      double mean_hue = mean_hsv[0];

      std::string class_id = "";
      if (area >= apple_min_area_ && circularity >= apple_min_circ_ &&
          aspect >= apple_aspect_min_ && aspect <= apple_aspect_max_ &&
          (mean_hue < 15 || mean_hue > 160)) {           // 红色苹果
        class_id = "apple";
      } else if (area >= banana_min_area_ && circularity <= banana_max_circ_ &&
                 aspect >= banana_aspect_min_ && aspect <= banana_aspect_max_ &&
                 mean_hue > 15 && mean_hue < 60) {        // 黄色香蕉
        class_id = "banana";
      }

      if (!class_id.empty()) {
        RCLCPP_INFO(this->get_logger(), 
          "Detected %s | area=%.0f, circ=%.3f, aspect=%.2f, mean_hue=%.1f", 
          class_id.c_str(), area, circularity, aspect, mean_hue);

        // 发布 Detection2D
        vision_msgs::msg::Detection2D det;
        det.header = msg->header;
        det.bbox.center.position.x = bbox.x + bbox.width / 2.0;
        det.bbox.center.position.y = bbox.y + bbox.height / 2.0;
        det.bbox.size_x = bbox.width;
        det.bbox.size_y = bbox.height;
        det.results.resize(1);
        det.results[0].hypothesis.class_id = class_id;
        detections.detections.push_back(det);

        // 在图像上画框和文字（关键修复！）
        cv::Scalar color = (class_id == "apple") ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
        cv::rectangle(image, bbox, color, 4);
        cv::putText(image, class_id, cv::Point(bbox.x, bbox.y - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 1.2, color, 3);

        // MarkerArray（保留，供3D视图使用）
        // ...（保持你原来的Marker代码，或暂时注释掉）
      }
    }

    pub_detections_->publish(detections);
    if (!marker_array.markers.empty()) {
      pub_markers_->publish(marker_array);
    }

    // 发布带标注的图像
    auto annotated_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
    pub_annotated_->publish(*annotated_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_detections_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_;   // 新增

  cv::Scalar hsv_lower_, hsv_upper_;
  int kernel_size_;
  double min_area_;
  double apple_min_circ_, apple_aspect_min_, apple_aspect_max_, apple_min_area_;
  double banana_max_circ_, banana_aspect_min_, banana_aspect_max_, banana_min_area_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectDetector>());
  rclcpp::shutdown();
  return 0;
}