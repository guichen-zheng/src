#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    cv::VideoCapture cap(0);  // 0 是默认摄像头ID
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    // 降分辨率以优化性能
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    // 创建窗口和轨迹条
    cv::namedWindow("HSV Adjust");
    int h_l = 0, h_u = 30, s_l = 48, s_u = 255, v_l = 80, v_u = 255;
    cv::createTrackbar("H Lower", "HSV Adjust", &h_l, 180);
    cv::createTrackbar("H Upper", "HSV Adjust", &h_u, 180);
    cv::createTrackbar("S Lower", "HSV Adjust", &s_l, 255);
    cv::createTrackbar("S Upper", "HSV Adjust", &s_u, 255);
    cv::createTrackbar("V Lower", "HSV Adjust", &v_l, 255);
    cv::createTrackbar("V Upper", "HSV Adjust", &v_u, 255);

    // 形状阈值
    double min_area = 5000.0;
    double min_circularity = 0.2;
    double aspect_min = 0.5;
    double aspect_max = 2.0;

    // 帧率控制：1 FPS (1s一次)
    int delay = 1000;  // ms

    while (true) {
        auto start_frame = std::chrono::steady_clock::now();

        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        // 可选：水平翻转像镜子（如果指向反了）
        // cv::flip(frame, frame, 1);  // 取消注释如果需要

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Scalar lower(h_l, s_l, v_l);
        cv::Scalar upper(h_u, s_u, v_u);

        cv::Mat mask;
        cv::inRange(hsv, lower, upper, mask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));  // 小kernel优化
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::string direction;  // 默认空
        if (!contours.empty()) {
            auto largest_it = std::max_element(contours.begin(), contours.end(),
                                               [](const auto& a, const auto& b) {
                                                   return cv::contourArea(a) < cv::contourArea(b);
                                               });
            std::vector<cv::Point> largest = *largest_it;

            double area = cv::contourArea(largest);
            if (area >= min_area) {
                double perimeter = cv::arcLength(largest, true);
                double circularity = (perimeter > 0) ? (4 * CV_PI * area / (perimeter * perimeter)) : 0;
                if (circularity >= min_circularity) {
                    cv::Rect bbox = cv::boundingRect(largest);
                    double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
                    if (aspect_ratio >= aspect_min && aspect_ratio <= aspect_max) {
                        // 计算凸包
                        std::vector<cv::Point> hull;
                        cv::convexHull(largest, hull);

                        // 找极端点
                        auto left_it = std::min_element(hull.begin(), hull.end(),
                                                        [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
                        cv::Point extreme_left = *left_it;

                        auto right_it = std::max_element(hull.begin(), hull.end(),
                                                         [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
                        cv::Point extreme_right = *right_it;

                        auto top_it = std::min_element(hull.begin(), hull.end(),
                                                       [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; });
                        cv::Point extreme_top = *top_it;

                        auto bottom_it = std::max_element(hull.begin(), hull.end(),
                                                          [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; });
                        cv::Point extreme_bottom = *bottom_it;

                        // 中心
                        int center_x = (extreme_left.x + extreme_right.x) / 2;
                        int center_y = (extreme_top.y + extreme_bottom.y) / 2;

                        // 判断指向（调整逻辑：如果左极端更远→左转，反之右转）
                        int left_dist = center_x - extreme_left.x;
                        int right_dist = extreme_right.x - center_x;
                        if (right_dist > left_dist) {
                            direction = "右转";
                        } else if (left_dist > right_dist) {
                            direction = "左转";
                        }

                        if (!direction.empty()) {
                            cv::drawContours(frame, {largest}, -1, cv::Scalar(0, 255, 0), 2);
                            cv::circle(frame, extreme_left, 5, cv::Scalar(255, 0, 0), -1);
                            cv::circle(frame, extreme_right, 5, cv::Scalar(0, 0, 255), -1);
                            cv::circle(frame, cv::Point(center_x, center_y), 5, cv::Scalar(0, 255, 255), -1);
                            cv::putText(frame, direction, cv::Point(bbox.x, bbox.y - 10),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                            std::cout << "手指向: " << direction << std::endl;
                        }
                    }
                }
            }
        }

        cv::imshow("Original with Detection", frame);
        cv::imshow("Mask", mask);

        auto end_frame = std::chrono::steady_clock::now();
        auto frame_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_frame - start_frame).count();
        std::cout << "本帧处理时间: " << frame_time << "ms" << std::endl;

        // 延时控制帧率
        int key = cv::waitKey(delay);
        if (key == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}