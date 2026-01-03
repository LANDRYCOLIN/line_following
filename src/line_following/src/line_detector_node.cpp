#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <string>
#include <vector>

class LineDetectorNode : public rclcpp::Node {
public:
  LineDetectorNode() : Node("line_detector_node") {
    // -------- Parameters --------
    image_topic_   = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    error_topic_   = declare_parameter<std::string>("error_topic", "/line/error");
    debug_topic_   = declare_parameter<std::string>("debug_topic", "/line/debug_image");

    threshold_     = declare_parameter<int>("threshold", 60);
    roi_ratio_     = declare_parameter<double>("roi_ratio", 0.5);
    morph_ksize_   = declare_parameter<int>("morph_ksize", 5);
    publish_debug_ = declare_parameter<bool>("publish_debug", true);

    // -------- Publishers --------
    error_pub_ = create_publisher<std_msgs::msg::Float32>(error_topic_, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_topic_, 1);

    // -------- Subscriber --------
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LineDetectorNode::onImage, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(),
      "LineDetector started. image_topic=%s, error_topic=%s, debug_topic=%s",
      image_topic_.c_str(), error_topic_.c_str(), debug_topic_.c_str());
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    const cv::Mat & frame = cv_ptr->image;
    if (frame.empty()) return;

    const int h = frame.rows;
    const int w = frame.cols;

    // ROI：下半部分
    const int roi_y = static_cast<int>(h * (1.0 - roi_ratio_));
    const int roi_h = h - roi_y;
    if (roi_y < 0 || roi_h <= 0) return;

    cv::Mat roi = frame(cv::Rect(0, roi_y, w, roi_h));

    // 灰度
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

    // 二值化（黑线->白）
    cv::Mat binary;
    cv::threshold(gray, binary, threshold_, 255, cv::THRESH_BINARY_INV);

    // 形态学开运算去噪
    int k = std::max(1, morph_ksize_);
    if (k % 2 == 0) k += 1;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {k, k});
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    // 找轮廓（最大区域当作线）
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    float error_norm = 0.0f; // [-1,1]
    bool found = false;

    int cx = -1, cy = -1;

    if (!contours.empty()) {
      auto max_it = std::max_element(
        contours.begin(), contours.end(),
        [](const auto & a, const auto & b) {
          return cv::contourArea(a) < cv::contourArea(b);
        }
      );

      cv::Moments m = cv::moments(*max_it);
      if (m.m00 > 1e-6) {
        cx = static_cast<int>(m.m10 / m.m00);
        cy = static_cast<int>(m.m01 / m.m00);

        // 映射回整图坐标
        cy += roi_y;

        const float center_x = 0.5f * w;
        const float e_px = static_cast<float>(cx) - center_x;
        error_norm = e_px / center_x; // 左负右正
        found = true;
      }
    }

    // 发布 error
    std_msgs::msg::Float32 e_msg;
    e_msg.data = found ? error_norm : 0.0f;
    error_pub_->publish(e_msg);

    // 发布 debug 图（可选）
    if (publish_debug_) {
      cv::Mat vis = frame.clone();
      cv::rectangle(vis, cv::Rect(0, roi_y, w, roi_h), cv::Scalar(0, 255, 255), 2);

      if (found) {
        cv::circle(vis, {cx, cy}, 6, {0, 0, 255}, -1);
        cv::line(vis, {w/2, h-1}, {cx, cy}, {255, 0, 0}, 2);
        cv::putText(vis, "error_norm: " + std::to_string(error_norm),
                    {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0,255,0}, 2);
      } else {
        cv::putText(vis, "line not found",
                    {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0,0,255}, 2);
      }

      auto out = cv_bridge::CvImage(msg->header, "bgr8", vis).toImageMsg();
      debug_pub_->publish(*out);
    }
  }

private:
  std::string image_topic_, error_topic_, debug_topic_;
  int threshold_;
  double roi_ratio_;
  int morph_ksize_;
  bool publish_debug_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
