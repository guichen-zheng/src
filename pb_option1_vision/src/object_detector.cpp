#include "pb_option1_vision/object_detector.hpp"
#include <iostream>
#include <algorithm>
#include <float.h>

namespace pb_option1_vision
{

YOLODetector::YOLODetector(const std::string& model_path, float conf_threshold, float nms_threshold)
  : conf_threshold_(conf_threshold), nms_threshold_(nms_threshold)
{
  net_ = cv::dnn::readNetFromONNX(model_path);
  if (net_.empty()) {
    throw std::runtime_error("无法加载ONNX模型: " + model_path);
  }
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

  // COCO 80类
  class_names_ = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
  };
}

std::vector<Detection> YOLODetector::detect(const cv::Mat& image)
{
  cv::Mat blob;
  cv::dnn::blobFromImage(image, blob, 1/255.0, cv::Size(640, 640), cv::Scalar(), true, false);
  net_.setInput(blob);
  std::vector<cv::Mat> outputs;
  net_.forward(outputs, net_.getUnconnectedOutLayersNames());

  return postprocess(image, outputs);
}

std::vector<Detection> YOLODetector::postprocess(const cv::Mat& frame, const std::vector<cv::Mat>& outputs)
{
    std::vector<Detection> detections;
    if (outputs.empty()) return detections;

    const cv::Mat& output = outputs[0]; // 形状应为 [1, 84, 8400]

    // --- 1. 检查并打印维度信息（用于调试）---
    if (output.dims != 3) {
        std::cerr << "错误：输出维度不是3，实际为 " << output.dims << std::endl;
        return detections;
    }
    int dimensions = output.size[1]; // 应为84
    int num_anchors = output.size[2]; // 应为8400
    std::cout << "简化模型输出形状: [1, " << dimensions << ", " << num_anchors << "]" << std::endl;

    if (dimensions != 84 || num_anchors == 0) {
        std::cerr << "错误：输出维度异常" << std::endl;
        return detections;
    }

    // --- 2. 核心修改：将数据转置为 [8400, 84] 以便按行处理 ---
    const float* data = output.ptr<float>();
    // 创建一个临时Mat指向原始数据，形状为 [84, 8400]
    cv::Mat output_buffer(dimensions, num_anchors, CV_32F, (float*)data);
    // 转置为 [8400, 84]
    cv::Mat output_transposed = output_buffer.t();
    float* pdata = (float*)output_transposed.data;

    int num_classes = dimensions - 4; // 80

    // --- 3. 遍历每个检测框（现在有8400行）---
    for (int i = 0; i < num_anchors; ++i) {
        float* row_ptr = pdata + i * dimensions;

        // 提取坐标 [cx, cy, w, h] (这些是模型直接预测的，需要解码)
        float cx = row_ptr[0];
        float cy = row_ptr[1];
        float w = row_ptr[2];
        float h = row_ptr[3];

        // --- 4. 类别分数处理（索引4-83）---
        float* scores_ptr = row_ptr + 4;
        float max_score = -FLT_MAX;
        int class_id = -1;
        for (int j = 0; j < num_classes; ++j) {
            if (scores_ptr[j] > max_score) {
                max_score = scores_ptr[j];
                class_id = j;
            }
        }

        // --- 5. 置信度过滤 ---
        if (max_score >= conf_threshold_ && class_id >= 0) {
            // 注意：YOLOv8的cx, cy, w, h 已经是相对于640x640的归一化值，并经过了sigmoid等处理。
            // 此处无需再应用复杂的锚框变换，直接按比例缩放到原图。
            float x1 = (cx - w/2) * frame.cols / 640.0f;
            float y1 = (cy - h/2) * frame.rows / 640.0f;
            float x2 = (cx + w/2) * frame.cols / 640.0f;
            float y2 = (cy + h/2) * frame.rows / 640.0f;

            // 边界裁剪
            x1 = std::max(0.0f, std::min(x1, (float)frame.cols));
            y1 = std::max(0.0f, std::min(y1, (float)frame.rows));
            x2 = std::max(0.0f, std::min(x2, (float)frame.cols));
            y2 = std::max(0.0f, std::min(y2, (float)frame.rows));

            cv::Rect box((int)x1, (int)y1, (int)(x2 - x1), (int)(y2 - y1));
            cv::Point2f center((x1 + x2) / 2, (y1 + y2) / 2);
            detections.push_back({class_id, max_score, box, center});
        }
    }

    // --- 6. NMS处理（与原代码相同）---
    if (detections.empty()) return detections;
    std::vector<int> indices;
    std::vector<cv::Rect> boxes;
    std::vector<float> confs;
    for (const auto& d : detections) {
        boxes.push_back(d.bbox);
        confs.push_back(d.confidence);
    }
    cv::dnn::NMSBoxes(boxes, confs, conf_threshold_, nms_threshold_, indices);

    std::vector<Detection> filtered;
    for (int idx : indices) {
        filtered.push_back(detections[idx]);
    }
    return filtered;
}

} // namespace pb_option1_vision