#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

class VerticalLineDetectorNode : public rclcpp::Node {
public:
  VerticalLineDetectorNode() : Node("vertical_line_detector_node") {
    binary_topic_ = declare_parameter<std::string>("binary_topic", "/line/binary_image");
    line_topic_ = declare_parameter<std::string>("line_topic", "/vertical_line/line");
    angle_topic_ = declare_parameter<std::string>("angle_topic", "/vertical_line/angle_deg");
    x_topic_ = declare_parameter<std::string>("x_topic", "/vertical_line/x_at_y_half");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/vertical_line/debug_image");

    morph_open_ksize_ = declare_parameter<int>("morph_open_ksize", 3);
    morph_close_ksize_ = declare_parameter<int>("morph_close_ksize", 5);
    border_margin_px_ = declare_parameter<int>("border_margin_px", 8);
    hough_threshold_ = declare_parameter<int>("hough_threshold", 12);
    hough_min_length_ = declare_parameter<int>("hough_min_length", 30);
    hough_max_gap_ = declare_parameter<int>("hough_max_gap", 40);
    max_abs_angle_deg_ = declare_parameter<double>("max_abs_angle_deg", 30.0);
    angle_penalty_ = declare_parameter<double>("angle_penalty", 2.0);
    publish_debug_ = declare_parameter<bool>("publish_debug", true);
    show_fps_overlay_ = declare_parameter<bool>("show_fps_overlay", true);
    fps_ema_alpha_ = declare_parameter<double>("fps_ema_alpha", 0.2);

    line_pub_ = create_publisher<geometry_msgs::msg::Point>(line_topic_, 10);
    angle_pub_ = create_publisher<std_msgs::msg::Float32>(angle_topic_, 10);
    x_pub_ = create_publisher<std_msgs::msg::Float32>(x_topic_, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_topic_, 1);

    binary_sub_ = create_subscription<sensor_msgs::msg::Image>(
      binary_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&VerticalLineDetectorNode::onBinaryImage, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "VerticalLineDetector started. binary_topic=%s, line_topic=%s, angle_topic=%s, x_topic=%s",
      binary_topic_.c_str(),
      line_topic_.c_str(),
      angle_topic_.c_str(),
      x_topic_.c_str());
  }

private:
  struct CandidateLine {
    cv::Point2f p1;
    cv::Point2f p2;
    double angle_deg;
    double length;
    double score;
  };

  void onBinaryImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    updateRealtimeFps();

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      publishInvalid(msg->header);
      return;
    }

    if (cv_ptr->image.empty()) {
      publishInvalid(msg->header);
      return;
    }

    cv::Mat gray;
    if (cv_ptr->image.channels() == 1) {
      gray = cv_ptr->image.clone();
    } else {
      cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    }

    cv::Mat binary;
    cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY);
    applyMorphology(binary);
    applyBorderMask(binary);

    CandidateLine best_line;
    const bool found = detectBestVerticalLine(binary, best_line);

    if (found) {
      publishDetection(msg->header, binary.size(), best_line);
    } else {
      publishInvalid(msg->header);
    }

    if (publish_debug_) {
      publishDebugImage(msg->header, binary, found, best_line);
    }
  }

  void updateRealtimeFps() {
    const auto now_tp = std::chrono::steady_clock::now();
    if (!fps_initialized_) {
      last_frame_tp_ = now_tp;
      fps_initialized_ = true;
      return;
    }

    const std::chrono::duration<double> dt = now_tp - last_frame_tp_;
    last_frame_tp_ = now_tp;
    if (dt.count() <= 1e-6) {
      return;
    }

    const double inst_fps = 1.0 / dt.count();
    const double alpha = std::clamp(fps_ema_alpha_, 0.01, 1.0);
    if (fps_value_ <= 0.0) {
      fps_value_ = inst_fps;
    } else {
      fps_value_ = alpha * inst_fps + (1.0 - alpha) * fps_value_;
    }
  }

  void applyMorphology(cv::Mat &binary) const {
    if (binary.empty()) {
      return;
    }

    if (morph_open_ksize_ > 1) {
      int k = std::max(3, morph_open_ksize_ | 1);
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {k, k});
      cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
    }

    if (morph_close_ksize_ > 1) {
      int k = std::max(3, morph_close_ksize_ | 1);
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {k, k});
      cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    }
  }

  void applyBorderMask(cv::Mat &binary) const {
    if (binary.empty()) {
      return;
    }
    const int margin = std::max(0, border_margin_px_);
    if (margin == 0) {
      return;
    }

    const int margin_x = std::min(margin, std::max(0, binary.cols / 2 - 1));
    const int margin_y = std::min(margin, std::max(0, binary.rows / 2 - 1));

    if (margin_y > 0) {
      binary.rowRange(0, margin_y).setTo(0);
      binary.rowRange(binary.rows - margin_y, binary.rows).setTo(0);
    }
    if (margin_x > 0) {
      binary.colRange(0, margin_x).setTo(0);
      binary.colRange(binary.cols - margin_x, binary.cols).setTo(0);
    }
  }

  bool detectBestVerticalLine(const cv::Mat &binary, CandidateLine &best_line) const {
    std::vector<cv::Vec4i> segments;
    cv::HoughLinesP(binary, segments, 1.0, CV_PI / 180.0,
                    std::max(1, hough_threshold_),
                    std::max(1, hough_min_length_),
                    std::max(0, hough_max_gap_));
    if (segments.empty()) {
      return false;
    }

    bool found = false;
    double best_score = -std::numeric_limits<double>::infinity();

    for (const auto & seg : segments) {
      cv::Point2f p1(seg[0], seg[1]);
      cv::Point2f p2(seg[2], seg[3]);
      if (p2.y < p1.y) {
        std::swap(p1, p2);
      }

      const double dx = static_cast<double>(p2.x - p1.x);
      const double dy = static_cast<double>(p2.y - p1.y);
      const double length = std::hypot(dx, dy);
      if (length < 1.0) {
        continue;
      }

      const double angle_deg = std::atan2(dx, dy) * 180.0 / CV_PI;
      if (std::abs(angle_deg) > max_abs_angle_deg_) {
        continue;
      }

      const double score = length - angle_penalty_ * std::abs(angle_deg);
      if (!found || score > best_score) {
        found = true;
        best_score = score;
        best_line = CandidateLine{p1, p2, angle_deg, length, score};
      }
    }

    return found;
  }

  void publishDetection(const std_msgs::msg::Header & header,
                        const cv::Size & image_size,
                        const CandidateLine & line) {
    const double y_query_px = 0.5 * static_cast<double>(std::max(0, image_size.height - 1));
    const double dy = static_cast<double>(line.p2.y - line.p1.y);
    const double dx = static_cast<double>(line.p2.x - line.p1.x);

    if (std::abs(dy) < 1e-6) {
      publishInvalid(header);
      return;
    }

    const double x_query_px = static_cast<double>(line.p1.x) +
      (y_query_px - static_cast<double>(line.p1.y)) * dx / dy;
    const double x_norm = x_query_px / static_cast<double>(std::max(1, image_size.width - 1));

    geometry_msgs::msg::Point line_msg;
    line_msg.x = std::clamp(x_norm, 0.0, 1.0);
    line_msg.y = 0.5;
    line_msg.z = line.angle_deg;
    line_pub_->publish(line_msg);

    std_msgs::msg::Float32 angle_msg;
    angle_msg.data = static_cast<float>(line.angle_deg);
    angle_pub_->publish(angle_msg);

    std_msgs::msg::Float32 x_msg;
    x_msg.data = static_cast<float>(line_msg.x);
    x_pub_->publish(x_msg);
  }

  void publishInvalid(const std_msgs::msg::Header & header) {
    (void)header;
    const double nan = std::numeric_limits<double>::quiet_NaN();

    geometry_msgs::msg::Point line_msg;
    line_msg.x = nan;
    line_msg.y = nan;
    line_msg.z = nan;
    line_pub_->publish(line_msg);

    std_msgs::msg::Float32 angle_msg;
    angle_msg.data = std::numeric_limits<float>::quiet_NaN();
    angle_pub_->publish(angle_msg);

    std_msgs::msg::Float32 x_msg;
    x_msg.data = std::numeric_limits<float>::quiet_NaN();
    x_pub_->publish(x_msg);
  }

  void publishDebugImage(const std_msgs::msg::Header & header,
                         const cv::Mat &binary,
                         bool found,
                         const CandidateLine & line) {
    cv::Mat vis;
    cv::cvtColor(binary, vis, cv::COLOR_GRAY2BGR);

    const int h = binary.rows;
    const int w = binary.cols;
    const int y_half = static_cast<int>(std::lround(0.5 * static_cast<double>(std::max(0, h - 1))));
    cv::line(vis, {0, y_half}, {std::max(0, w - 1), y_half}, cv::Scalar(255, 128, 0), 1);

    if (found) {
      cv::line(vis, line.p1, line.p2, cv::Scalar(0, 255, 0), 2);
      const double dy = static_cast<double>(line.p2.y - line.p1.y);
      const double dx = static_cast<double>(line.p2.x - line.p1.x);
      if (std::abs(dy) > 1e-6) {
        const double x_half = static_cast<double>(line.p1.x) +
          (static_cast<double>(y_half) - static_cast<double>(line.p1.y)) * dx / dy;
        const int x_half_i = static_cast<int>(
          std::lround(std::clamp(x_half, 0.0, static_cast<double>(std::max(0, w - 1)))));
        cv::drawMarker(vis, {x_half_i, y_half}, cv::Scalar(0, 0, 255),
                       cv::MARKER_CROSS, 20, 2);
      }

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3)
          << "angle=" << line.angle_deg << " deg";
      cv::putText(vis, oss.str(), {20, 30},
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    } else {
      cv::putText(vis, "vertical line not found", {20, 30},
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }

    if (show_fps_overlay_ && fps_value_ > 0.0) {
      std::ostringstream fps_ss;
      fps_ss << std::fixed << std::setprecision(1) << "FPS: " << fps_value_;
      cv::putText(vis, fps_ss.str(), {20, 60},
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    }

    auto debug_msg = cv_bridge::CvImage(header, "bgr8", vis).toImageMsg();
    debug_pub_->publish(*debug_msg);
  }

  std::string binary_topic_;
  std::string line_topic_;
  std::string angle_topic_;
  std::string x_topic_;
  std::string debug_topic_;

  int morph_open_ksize_;
  int morph_close_ksize_;
  int border_margin_px_;
  int hough_threshold_;
  int hough_min_length_;
  int hough_max_gap_;
  double max_abs_angle_deg_;
  double angle_penalty_;
  bool publish_debug_;
  bool show_fps_overlay_;
  double fps_ema_alpha_;

  bool fps_initialized_{false};
  double fps_value_{0.0};
  std::chrono::steady_clock::time_point last_frame_tp_{};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr binary_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr line_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr x_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VerticalLineDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
