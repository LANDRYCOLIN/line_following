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

    hough_threshold_ = declare_parameter<int>("hough_threshold", 12);
    hough_min_length_ = declare_parameter<int>("hough_min_length", 20); // 降低要求，避免弯道切线过短被滤除
    hough_max_gap_ = declare_parameter<int>("hough_max_gap", 40);
    
    // 如果你在 launch 里没改，这里默认是 85度
    max_abs_angle_deg_ = declare_parameter<double>("max_abs_angle_deg", 85.0); 
    publish_debug_ = declare_parameter<bool>("publish_debug", true);
    show_fps_overlay_ = declare_parameter<bool>("show_fps_overlay", true);
    fps_ema_alpha_ = declare_parameter<double>("fps_ema_alpha", 0.2);
    output_ema_alpha_ = declare_parameter<double>("output_ema_alpha", 0.8);

    line_pub_ = create_publisher<geometry_msgs::msg::Point>(line_topic_, 10);
    angle_pub_ = create_publisher<std_msgs::msg::Float32>(angle_topic_, 10);
    x_pub_ = create_publisher<std_msgs::msg::Float32>(x_topic_, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_topic_, 1);

    binary_sub_ = create_subscription<sensor_msgs::msg::Image>(
      binary_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VerticalLineDetectorNode::onBinaryImage, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "VerticalLineDetector (Local Tangent Edition) started.");
  }

private:
  struct CandidateLine {
    double angle_deg;
    double x_at_center;
  };

  void onBinaryImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    updateRealtimeFps();

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (...) {
      publishInvalid(msg->header);
      return;
    }

    if (cv_ptr->image.empty()) {
      publishInvalid(msg->header);
      return;
    }

    cv::Mat binary;
    if (cv_ptr->image.channels() == 1) {
      binary = cv_ptr->image.clone();
    } else {
      cv::cvtColor(cv_ptr->image, binary, cv::COLOR_BGR2GRAY);
    }
    cv::threshold(binary, binary, 127, 255, cv::THRESH_BINARY);

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
    if (dt.count() > 1e-6) {
      const double inst_fps = 1.0 / dt.count();
      fps_value_ = fps_value_ <= 0.0 ? inst_fps : fps_ema_alpha_ * inst_fps + (1.0 - fps_ema_alpha_) * fps_value_;
    }
  }

  bool detectBestVerticalLine(const cv::Mat &binary, CandidateLine &best_line) {
    // 1. 极其鲁棒的骨架提取 (免疫 T型路口干扰)
    cv::Mat binary_01;
    binary.convertTo(binary_01, CV_8U, 1.0 / 255.0);
    cv::Mat dist;
    cv::distanceTransform(binary_01, dist, cv::DIST_L2, 3);
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
    cv::Mat dist_dil;
    cv::dilate(dist, dist_dil, kernel);
    cv::Mat ridge;
    cv::compare(dist, dist_dil, ridge, cv::CMP_GE);
    
    cv::Mat strong;
    // 使用极低的绝对阈值(3.0)，只要线宽 > 6像素，骨架绝对不会断！
    cv::threshold(dist, strong, 3.0, 255, cv::THRESH_BINARY);
    strong.convertTo(strong, CV_8U);
    cv::Mat centerline;
    cv::bitwise_and(ridge, strong, centerline);

    // 2. Hough 提取局部切线段
    std::vector<cv::Vec4i> segments;
    cv::HoughLinesP(centerline, segments, 1.0, CV_PI / 180.0,
                    std::max(1, hough_threshold_),
                    std::max(1, hough_min_length_),
                    std::max(0, hough_max_gap_));

    if (segments.empty()) {
      filter_initialized_ = false; return false;
    }

    // 3. 切线评分：寻找最靠近画面中心的引导切线
    bool found = false;
    double best_score = -1e9;
    double best_raw_angle = 0.0;
    double best_raw_x = 0.0;
    const double y_half = binary.rows / 2.0;

    for (const auto & seg : segments) {
      cv::Point2f p1(seg[0], seg[1]);
      cv::Point2f p2(seg[2], seg[3]);
      if (p2.y < p1.y) std::swap(p1, p2);

      double dx = p2.x - p1.x;
      double dy = p2.y - p1.y;
      double length = std::hypot(dx, dy);
      if (length < 5.0) continue;

      double angle_deg = std::atan2(dx, dy) * 180.0 / CV_PI;
      // 这里的 max_abs_angle_deg_ 取决于你有没有修改 launch 文件！
      if (std::abs(angle_deg) > max_abs_angle_deg_) continue;

      // 打分逻辑：距离 y=0.5 越近越好，长度越长越好
      double mid_y = (p1.y + p2.y) / 2.0;
      double dist_y = std::abs(mid_y - y_half);
      double score = length - 0.8 * dist_y;

      if (score > best_score) {
        best_score = score;
        best_raw_angle = angle_deg;
        if (std::abs(dy) < 1e-6) {
          best_raw_x = p1.x;
        } else {
          best_raw_x = p1.x + (y_half - p1.y) * dx / dy;
        }
        found = true;
      }
    }

    if (!found) {
      filter_initialized_ = false; return false;
    }

    // 4. EMA 滤波重构
    if (!filter_initialized_) {
      smoothed_x_ = best_raw_x;
      smoothed_angle_ = best_raw_angle;
      filter_initialized_ = true;
    } else {
      double alpha = std::clamp(output_ema_alpha_, 0.01, 1.0);
      smoothed_x_ = alpha * best_raw_x + (1.0 - alpha) * smoothed_x_;
      smoothed_angle_ = alpha * best_raw_angle + (1.0 - alpha) * smoothed_angle_;
    }

    best_line = CandidateLine{smoothed_angle_, smoothed_x_};
    return true;
  }

  void publishDetection(const std_msgs::msg::Header & header, const cv::Size & image_size, const CandidateLine & line) {
    const double x_norm = line.x_at_center / static_cast<double>(std::max(1, image_size.width - 1));

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
    const double nan = std::numeric_limits<double>::quiet_NaN();
    geometry_msgs::msg::Point line_msg;
    line_msg.x = nan; line_msg.y = nan; line_msg.z = nan;
    line_pub_->publish(line_msg);

    std_msgs::msg::Float32 f_msg; f_msg.data = std::numeric_limits<float>::quiet_NaN();
    angle_pub_->publish(f_msg); x_pub_->publish(f_msg);
    filter_initialized_ = false;
  }

  void publishDebugImage(const std_msgs::msg::Header & header, const cv::Mat &binary, bool found, const CandidateLine & line) {
    cv::Mat vis;
    cv::cvtColor(binary, vis, cv::COLOR_GRAY2BGR);
    
    const int h = vis.rows; const int w = vis.cols;
    const int y_half = static_cast<int>(std::lround(0.5 * std::max(0, h - 1)));
    cv::line(vis, {0, y_half}, {std::max(0, w - 1), y_half}, cv::Scalar(255, 128, 0), 1);

    if (found) {
      // 绝佳的视觉反馈：绘制固定长度的“切线向量/方向箭头”
      double rad = line.angle_deg * CV_PI / 180.0;
      double vx = std::sin(rad);
      double vy = std::cos(rad);
      
      cv::Point2f center_pt(static_cast<float>(line.x_at_center), static_cast<float>(y_half));
      float vec_len = 100.0f; // 画一个总长度为 200 像素的切线
      cv::Point2f top_pt = center_pt - cv::Point2f(vx * vec_len, vy * vec_len);
      cv::Point2f bot_pt = center_pt + cv::Point2f(vx * vec_len, vy * vec_len);

      cv::line(vis, top_pt, bot_pt, cv::Scalar(0, 255, 0), 3); // 绿色的局部切线
      cv::drawMarker(vis, center_pt, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2); // 红色中心锚点

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3) << "angle=" << line.angle_deg << " deg";
      cv::putText(vis, oss.str(), {20, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    } else {
      cv::putText(vis, "vertical line not found", {20, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    
    if (show_fps_overlay_ && fps_value_ > 0.0) {
      std::ostringstream fps_ss; fps_ss << std::fixed << std::setprecision(1) << "FPS: " << fps_value_;
      cv::putText(vis, fps_ss.str(), {20, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    }
    
    debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", vis).toImageMsg());
  }

  std::string binary_topic_, line_topic_, angle_topic_, x_topic_, debug_topic_;
  int hough_threshold_, hough_min_length_, hough_max_gap_;
  double max_abs_angle_deg_, fps_ema_alpha_, output_ema_alpha_;
  bool publish_debug_, show_fps_overlay_;
  
  bool fps_initialized_{false};
  double fps_value_{0.0};
  std::chrono::steady_clock::time_point last_frame_tp_{};
  double smoothed_x_{-1.0}, smoothed_angle_{0.0};
  bool filter_initialized_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr binary_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr line_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_, x_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VerticalLineDetectorNode>());
  rclcpp::shutdown();
  return 0;
}