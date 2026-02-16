#ifndef PB_OPTION1_VISION__OBJECT_DETECTOR_HPP_
#define PB_OPTION1_VISION__OBJECT_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>
#include <memory>

namespace pb_option1_vision
{

struct Detection
{
  int class_id;
  float confidence;
  cv::Rect bbox;
  cv::Point2f center;   // 物体中心 (图像坐标系)
};

class YOLODetector
{
public:
  YOLODetector(const std::string& model_path, float conf_threshold, float nms_threshold = 0.45);
  ~YOLODetector() = default;

  // 执行检测，返回检测结果列表
  std::vector<Detection> detect(const cv::Mat& image);

  // 获取类别名称列表（假设模型输出索引对应COCO）
  const std::vector<std::string>& getClassNames() const { return class_names_; }

private:
  cv::dnn::Net net_;
  float conf_threshold_;
  float nms_threshold_;
  std::vector<std::string> class_names_;

  // 后处理
  std::vector<Detection> postprocess(const cv::Mat& frame, const std::vector<cv::Mat>& outputs);
};

}  // namespace pb_option1_vision

#endif  // PB_OPTION1_VISION__OBJECT_DETECTOR_HPP_