#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>  // 新增

class ObjectDetector : public rclcpp::Node {
public:
  ObjectDetector() : Node("object_detector") {
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", 10, std::bind(&ObjectDetector::imageCallback, this, std::placeholders::_1));

    pub_detections_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("/detected_objects", 10);

    // 新增：用于在 RViz 显示边界框的 MarkerArray
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detection_markers", 10);

    // 加载 YAML 参数
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
    cv::Mat hsv;
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, hsv_lower_, hsv_upper_, mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size_, kernel_size_));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    vision_msgs::msg::Detection2DArray detections;
    detections.header = msg->header;

    visualization_msgs::msg::MarkerArray marker_array;

    if (!contours.empty()) {
      auto largest_it = std::max_element(contours.begin(), contours.end(),
                                         [](const auto& a, const auto& b) { return cv::contourArea(a) < cv::contourArea(b); });
      std::vector<cv::Point> largest = *largest_it;

      double area = cv::contourArea(largest);
      if (area < min_area_) return;

      double perimeter = cv::arcLength(largest, true);
      double circularity = (perimeter > 0) ? (4 * CV_PI * area / (perimeter * perimeter)) : 0;

      cv::Rect bbox = cv::boundingRect(largest);
      double aspect = static_cast<double>(bbox.width) / bbox.height;

      std::string class_id;
      if (area >= apple_min_area_ && circularity >= apple_min_circ_ &&
          aspect >= apple_aspect_min_ && aspect <= apple_aspect_max_) {
        class_id = "apple";
      } else if (area >= banana_min_area_ && circularity <= banana_max_circ_ &&
                 aspect >= banana_aspect_min_ && aspect <= banana_aspect_max_) {
        class_id = "banana";
      }

      if (!class_id.empty()) {
        // 发布 Detection2DArray
        vision_msgs::msg::Detection2D det;
        det.header = msg->header;
        det.bbox.center.position.x = bbox.x + bbox.width / 2.0f;
        det.bbox.center.position.y = bbox.y + bbox.height / 2.0f;
        det.bbox.size_x = bbox.width;
        det.bbox.size_y = bbox.height;
        det.results.resize(1);
        det.results[0].hypothesis.class_id = class_id;
        detections.detections.push_back(det);

        // 发布 Marker（RViz 可视化）
        visualization_msgs::msg::Marker marker;
        marker.header = det.header;
        marker.ns = "detections";
        marker.id = marker_array.markers.size();
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.005;  // 线宽
        marker.lifetime = rclcpp::Duration::from_seconds(1.0);

        // 颜色：苹果绿色，香蕉橙色
        if (class_id == "apple") {
          marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
        } else {
          marker.color.r = 1.0f; marker.color.g = 0.5f; marker.color.b = 0.0f;
        }
        marker.color.a = 1.0f;

        // 画矩形框（5个点闭合）
        geometry_msgs::msg::Point p[5];
        float cx = det.bbox.center.position.x;
        float cy = det.bbox.center.position.y;
        float half_w = det.bbox.size_x / 2.0f;
        float half_h = det.bbox.size_y / 2.0f;

        p[0].x = cx - half_w; p[0].y = cy - half_h; p[0].z = 0.0;
        p[1].x = cx + half_w; p[1].y = cy - half_h; p[1].z = 0.0;
        p[2].x = cx + half_w; p[2].y = cy + half_h; p[2].z = 0.0;
        p[3].x = cx - half_w; p[3].y = cy + half_h; p[3].z = 0.0;
        p[4] = p[0];  // 闭合

        for (int i = 0; i < 5; ++i) {
          marker.points.push_back(p[i]);
        }

        marker_array.markers.push_back(marker);

        // 加文字标签（可选）
        visualization_msgs::msg::Marker text;
        text = marker;
        text.id = marker.id + 1000;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.pose.position.x = cx;
        text.pose.position.y = cy - half_h - 20;
        text.pose.position.z = 0.0;
        text.scale.z = 0.4;
        text.text = class_id;
        text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f;
        text.color.a = 1.0f;
        marker_array.markers.push_back(text);
      }
    }

    pub_detections_->publish(detections);
    if (!marker_array.markers.empty()) {
      pub_markers_->publish(marker_array);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_detections_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;  // 新增

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