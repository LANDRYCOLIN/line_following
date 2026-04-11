#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

class CornerDetectorNode : public rclcpp::Node {
public:
  CornerDetectorNode() : Node("corner_detector_node") {
    // -------- Parameters --------
    image_topic_   = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    error_topic_   = declare_parameter<std::string>("error_topic", "/line/error");
    debug_topic_   = declare_parameter<std::string>("debug_topic", "/line/debug_image");
    binary_topic_  = declare_parameter<std::string>("binary_topic", "/line/binary_image");
    corner_topic_  = declare_parameter<std::string>("corner_topic", "/line/corner");

    threshold_     = declare_parameter<int>("threshold", 210);
    auto_threshold_ = declare_parameter<bool>("auto_threshold", true);
    auto_thresh_k_  = declare_parameter<double>("auto_thresh_k", 2.0);
    auto_thresh_min_ = declare_parameter<int>("auto_thresh_min", 160);
    auto_thresh_max_ = declare_parameter<int>("auto_thresh_max", 235);
    roi_ratio_     = declare_parameter<double>("roi_ratio", 1.0);
    morph_ksize_   = declare_parameter<int>("morph_ksize", 5);
    close_ksize_   = declare_parameter<int>("close_ksize", 9);
    open_ksize_    = declare_parameter<int>("open_ksize", 5);
    blur_ksize_    = declare_parameter<int>("blur_ksize", 5);
    bilateral_d_   = declare_parameter<int>("bilateral_d", 0);
    bilateral_sigma_color_ = declare_parameter<double>("bilateral_sigma_color", 25.0);
    bilateral_sigma_space_ = declare_parameter<double>("bilateral_sigma_space", 25.0);
    show_fps_overlay_ = declare_parameter<bool>("show_fps_overlay", true);
    fps_ema_alpha_ = declare_parameter<double>("fps_ema_alpha", 0.2);
    publish_debug_ = declare_parameter<bool>("publish_debug", true);
    publish_binary_debug_ = declare_parameter<bool>("publish_binary_debug", true);

    intersection_margin_  = declare_parameter<int>("intersection_margin", 2);
    border_margin_px_     = declare_parameter<int>("border_margin_px", 60);
    centerline_ratio_     = declare_parameter<double>("centerline_ratio", 0.55);
    centerline_use_nms_   = declare_parameter<bool>("centerline_use_nms", true);
    centerline_nms_ksize_ = declare_parameter<int>("centerline_nms_ksize", 3);
    hough_threshold_      = declare_parameter<int>("hough_threshold", 30);
    hough_min_length_     = declare_parameter<int>("hough_min_length", 20);
    hough_max_gap_        = declare_parameter<int>("hough_max_gap", 20);
    corner_angle_min_deg_ = declare_parameter<double>("corner_angle_min_deg", 60.0);
    corner_angle_max_deg_ = declare_parameter<double>("corner_angle_max_deg", 120.0);
    corner_max_dist_px_   = declare_parameter<int>("corner_max_dist_px", 15);
    corner_len_weight_    = declare_parameter<double>("corner_len_weight", 1.0);
    corner_angle_weight_  = declare_parameter<double>("corner_angle_weight", 1.0);
    corner_dist_weight_   = declare_parameter<double>("corner_dist_weight", 1.5);

    // -------- Publishers --------
    error_pub_ = create_publisher<std_msgs::msg::Float32>(error_topic_, 10);
    corner_pub_ = create_publisher<geometry_msgs::msg::Point>(corner_topic_, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_topic_, 1);
    binary_pub_ = create_publisher<sensor_msgs::msg::Image>(binary_topic_, 1);

    // -------- Subscriber --------
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CornerDetectorNode::onImage, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(),
      "CornerDetector started. image_topic=%s, error_topic=%s, corner_topic=%s, debug_topic=%s, binary_topic=%s",
      image_topic_.c_str(), error_topic_.c_str(), corner_topic_.c_str(),
      debug_topic_.c_str(), binary_topic_.c_str());
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    updateRealtimeFps();

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

    // ROI：可根据参数裁剪高度
    const int roi_y = static_cast<int>(h * (1.0 - roi_ratio_));
    const int roi_h = h - roi_y;
    if (roi_y < 0 || roi_h <= 0) return;

    const cv::Rect roi_rect(0, roi_y, w, roi_h);
    cv::Mat roi = frame(roi_rect);

    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

    applyPreFilters(gray);

    cv::Mat binary;
    double threshold_value = static_cast<double>(threshold_);
    if (auto_threshold_) {
      cv::Scalar mean, stddev;
      cv::meanStdDev(gray, mean, stddev);
      threshold_value = mean[0] + auto_thresh_k_ * stddev[0];
      threshold_value = std::clamp(threshold_value,
                                   static_cast<double>(auto_thresh_min_),
                                   static_cast<double>(auto_thresh_max_));
    }
    cv::threshold(gray, binary, threshold_value, 255, cv::THRESH_BINARY);

    int k = std::max(1, morph_ksize_);
    if (k % 2 == 0) k += 1;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {k, k});
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    if (close_ksize_ > 1) {
      int ck = close_ksize_;
      if (ck % 2 == 0) ck += 1;
      ck = std::max(3, ck);
      cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, {ck, ck});
      cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, close_kernel);
    }
    if (open_ksize_ > 1) {
      int ok = open_ksize_;
      if (ok % 2 == 0) ok += 1;
      ok = std::max(3, ok);
      cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, {ok, ok});
      cv::morphologyEx(binary, binary, cv::MORPH_OPEN, open_kernel);
    }

    applyBorderMask(binary);

    if (publish_binary_debug_) {
      auto bin_msg = cv_bridge::CvImage(msg->header, "mono8", binary).toImageMsg();
      binary_pub_->publish(*bin_msg);
    }

    cv::Mat centerline;
    centerline_width_ = binary.cols;
    centerline_height_ = binary.rows;
    const bool has_centerline = computeCenterline(binary, centerline);

    std::vector<cv::Vec4i> segments;
    bool detected = false;
    if (has_centerline) {
      detected = detectLineSegments(centerline, segments);
    }

    cv::Point best_corner_roi;
    int best_seg_a = -1;
    int best_seg_b = -1;
    const bool found_corner = detected &&
      computeCornerFromSegments(segments, best_corner_roi, best_seg_a, best_seg_b);

    int cx = -1;
    int cy = -1;
    float error_norm = 0.0f;
    double norm_x = std::numeric_limits<double>::quiet_NaN();
    double norm_y = std::numeric_limits<double>::quiet_NaN();

    if (found_corner) {
      cx = best_corner_roi.x;
      cy = best_corner_roi.y + roi_y;

      const float center_x = 0.5f * w;
      const float e_px = static_cast<float>(cx) - center_x;
      error_norm = e_px / center_x;

      const double denom_x = static_cast<double>(std::max(1, w - 1));
      const double denom_y = static_cast<double>(std::max(1, h - 1));
      norm_x = std::max(0.0, std::min(1.0, static_cast<double>(cx) / denom_x));
      norm_y = std::max(0.0, std::min(1.0, static_cast<double>(cy) / denom_y));
    }

    std_msgs::msg::Float32 e_msg;
    e_msg.data = found_corner ? error_norm : 0.0f;
    error_pub_->publish(e_msg);

    geometry_msgs::msg::Point corner_msg;
    if (found_corner) {
      corner_msg.x = norm_x;
      corner_msg.y = norm_y;
      corner_msg.z = 0.0;
    } else {
      corner_msg.x = std::numeric_limits<double>::quiet_NaN();
      corner_msg.y = std::numeric_limits<double>::quiet_NaN();
      corner_msg.z = 0.0;
    }
    corner_pub_->publish(corner_msg);

    if (publish_debug_) {
      cv::Mat vis = frame.clone();
      cv::rectangle(vis, roi_rect, cv::Scalar(0, 255, 255), 2);

      if (has_centerline) {
        cv::Mat vis_roi = vis(roi_rect);
        vis_roi.setTo(cv::Scalar(0, 200, 255), centerline);
      }

      if (detected) {
        for (size_t i = 0; i < segments.size(); ++i) {
          const auto & seg = segments[i];
          const bool is_best = (static_cast<int>(i) == best_seg_a ||
                                static_cast<int>(i) == best_seg_b);
          const cv::Scalar color = is_best ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 255, 0);
          const int thickness = is_best ? 2 : 1;
          cv::line(vis,
                   {seg[0], seg[1] + roi_y},
                   {seg[2], seg[3] + roi_y},
                   color, thickness);
        }
      }

      if (found_corner) {
        cv::Point corner_px(cx, cy);
        cv::drawMarker(vis, corner_px, {0, 0, 255}, cv::MARKER_TILTED_CROSS, 26, 3);
        cv::line(vis, corner_px, {corner_px.x, roi_y}, {0, 150, 255}, 1);
        cv::line(vis, corner_px, {0, corner_px.y}, {0, 150, 255}, 1);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "corner (x,y) = (" << norm_x << ", " << norm_y << ")";
        cv::putText(vis, oss.str(), {20, 40},
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
      } else {
        cv::putText(vis, "corner not found",
                    {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255}, 2);
      }

      if (show_fps_overlay_ && fps_value_ > 0.0) {
        std::ostringstream fps_ss;
        fps_ss << std::fixed << std::setprecision(1) << "FPS: " << fps_value_;
        const std::string fps_text = fps_ss.str();
        int baseline = 0;
        const cv::Size text_size = cv::getTextSize(
          fps_text, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
        const int margin = 16;
        const cv::Point text_org(w - text_size.width - margin, margin + text_size.height);
        const cv::Rect bg_rect(
          text_org.x - 8,
          text_org.y - text_size.height - 6,
          text_size.width + 16,
          text_size.height + 12);
        cv::rectangle(vis, bg_rect, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(vis, fps_text, text_org,
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
      }

      auto out = cv_bridge::CvImage(msg->header, "bgr8", vis).toImageMsg();
      debug_pub_->publish(*out);
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

  bool computeCenterline(const cv::Mat &binary, cv::Mat &centerline) const {
    if (binary.empty()) {
      return false;
    }
    cv::Mat binary_01;
    binary.convertTo(binary_01, CV_8U, 1.0 / 255.0);
    cv::Mat dist;
    cv::distanceTransform(binary_01, dist, cv::DIST_L2, 3);
    double max_val = 0.0;
    cv::minMaxLoc(dist, nullptr, &max_val);
    if (max_val <= 0.0) {
      return false;
    }

    const double thr = max_val * std::clamp(centerline_ratio_, 0.1, 0.9);
    if (centerline_use_nms_) {
      int ksize = std::max(3, centerline_nms_ksize_ | 1);
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {ksize, ksize});
      cv::Mat dist_dil;
      cv::dilate(dist, dist_dil, kernel);
      cv::Mat ridge;
      cv::compare(dist, dist_dil, ridge, cv::CMP_GE);
      cv::Mat strong;
      cv::threshold(dist, strong, thr, 255, cv::THRESH_BINARY);
      strong.convertTo(strong, CV_8U);
      cv::bitwise_and(ridge, strong, centerline);
    } else {
      cv::threshold(dist, centerline, thr, 255, cv::THRESH_BINARY);
      centerline.convertTo(centerline, CV_8U);
    }
    return cv::countNonZero(centerline) > 0;
  }

  bool detectLineSegments(const cv::Mat &edge, std::vector<cv::Vec4i> &segments) const {
    if (edge.empty()) {
      return false;
    }
    segments.clear();
    cv::HoughLinesP(edge, segments, 1.0, CV_PI / 180.0,
                    std::max(1, hough_threshold_),
                    std::max(1, hough_min_length_),
                    std::max(0, hough_max_gap_));
    return !segments.empty();
  }

  static float distancePointToSegment(const cv::Point2f &p,
                                      const cv::Point2f &a,
                                      const cv::Point2f &b) {
    const cv::Point2f ab = b - a;
    const float ab2 = ab.dot(ab);
    if (ab2 <= 1e-6f) {
      return cv::norm(p - a);
    }
    const float t = std::clamp((p - a).dot(ab) / ab2, 0.0f, 1.0f);
    const cv::Point2f proj = a + t * ab;
    return cv::norm(p - proj);
  }

  bool computeCornerFromSegments(const std::vector<cv::Vec4i> &segments,
                                 cv::Point &intersection,
                                 int &seg_a,
                                 int &seg_b) const {
    if (segments.size() < 2) {
      return false;
    }

    const float min_angle = static_cast<float>(corner_angle_min_deg_);
    const float max_angle = static_cast<float>(corner_angle_max_deg_);
    const float max_dist = static_cast<float>(std::max(1, corner_max_dist_px_));
    const int max_x = std::max(0, centerline_width_ - 1);
    const int max_y = std::max(0, centerline_height_ - 1);

    double best_score = -1e9;
    cv::Point best_pt;
    int best_a = -1;
    int best_b = -1;

    for (size_t i = 0; i < segments.size(); ++i) {
      const auto &s1 = segments[i];
      const cv::Point2f p1(s1[0], s1[1]);
      const cv::Point2f p2(s1[2], s1[3]);
      const cv::Point2f d1 = p2 - p1;
      const float len1 = cv::norm(d1);
      if (len1 < 1.0f) {
        continue;
      }
      const cv::Point2f u1 = d1 * (1.0f / len1);

      for (size_t j = i + 1; j < segments.size(); ++j) {
        const auto &s2 = segments[j];
        const cv::Point2f p3(s2[0], s2[1]);
        const cv::Point2f p4(s2[2], s2[3]);
        const cv::Point2f d2 = p4 - p3;
        const float len2 = cv::norm(d2);
        if (len2 < 1.0f) {
          continue;
        }
        const cv::Point2f u2 = d2 * (1.0f / len2);

        float dot = std::fabs(u1.dot(u2));
        dot = std::clamp(dot, 0.0f, 1.0f);
        const float angle = std::acos(dot) * 180.0f / static_cast<float>(CV_PI);
        if (angle < min_angle || angle > max_angle) {
          continue;
        }

        const float denom = d1.x * d2.y - d1.y * d2.x;
        if (std::fabs(denom) < 1e-6f) {
          continue;
        }
        const cv::Point2f diff = p3 - p1;
        const float t = (diff.x * d2.y - diff.y * d2.x) / denom;
        const cv::Point2f pt = p1 + t * d1;

        if (pt.x < 0.0f || pt.y < 0.0f || pt.x > max_x || pt.y > max_y) {
          continue;
        }

        const float dist1 = distancePointToSegment(pt, p1, p2);
        const float dist2 = distancePointToSegment(pt, p3, p4);
        if (dist1 > max_dist || dist2 > max_dist) {
          continue;
        }

        const double score = corner_len_weight_ * (len1 + len2)
          - corner_angle_weight_ * std::fabs(90.0 - angle)
          - corner_dist_weight_ * (dist1 + dist2);

        if (score > best_score) {
          best_score = score;
          best_pt = cv::Point(cvRound(pt.x), cvRound(pt.y));
          best_a = static_cast<int>(i);
          best_b = static_cast<int>(j);
        }
      }
    }

    if (best_a < 0 || best_b < 0) {
      return false;
    }

    intersection = best_pt;
    const int min_x = std::max(0, intersection_margin_);
    const int min_y = std::max(0, intersection_margin_);
    intersection.x = std::clamp(intersection.x, min_x, max_x);
    intersection.y = std::clamp(intersection.y, min_y, max_y);
    seg_a = best_a;
    seg_b = best_b;
    return true;
  }

  void applyPreFilters(cv::Mat &gray) const {
    if (gray.empty()) {
      return;
    }
    if (blur_ksize_ > 1) {
      int ksize = blur_ksize_;
      if (ksize % 2 == 0) {
        ksize += 1;
      }
      ksize = std::max(3, ksize);
      cv::GaussianBlur(gray, gray, cv::Size(ksize, ksize), 0);
    }
    if (bilateral_d_ > 0) {
      cv::bilateralFilter(gray, gray, bilateral_d_,
                          std::max(1.0, bilateral_sigma_color_),
                          std::max(1.0, bilateral_sigma_space_));
    }
  }

  void applyBorderMask(cv::Mat &mask) const {
    if (mask.empty()) {
      return;
    }
    const int margin = std::max(0, border_margin_px_);
    if (margin == 0) {
      return;
    }
    const int max_margin_x = std::max(0, mask.cols / 2 - 1);
    const int max_margin_y = std::max(0, mask.rows / 2 - 1);
    const int margin_x = std::min(margin, max_margin_x);
    const int margin_y = std::min(margin, max_margin_y);

    if (margin_y > 0) {
      mask.rowRange(0, margin_y).setTo(0);
      mask.rowRange(mask.rows - margin_y, mask.rows).setTo(0);
    }
    if (margin_x > 0) {
      mask.colRange(0, margin_x).setTo(0);
      mask.colRange(mask.cols - margin_x, mask.cols).setTo(0);
    }
  }

private:
  std::string image_topic_, error_topic_, debug_topic_, binary_topic_, corner_topic_;
  int threshold_;
  bool auto_threshold_;
  double auto_thresh_k_;
  int auto_thresh_min_;
  int auto_thresh_max_;
  double roi_ratio_;
  int morph_ksize_;
  int close_ksize_;
  int open_ksize_;
  int blur_ksize_;
  int bilateral_d_;
  double bilateral_sigma_color_;
  double bilateral_sigma_space_;
  bool show_fps_overlay_;
  double fps_ema_alpha_;
  bool publish_debug_;
  bool publish_binary_debug_;
  int intersection_margin_;
  int border_margin_px_;
  double centerline_ratio_;
  bool centerline_use_nms_;
  int centerline_nms_ksize_;
  int hough_threshold_;
  int hough_min_length_;
  int hough_max_gap_;
  double corner_angle_min_deg_;
  double corner_angle_max_deg_;
  int corner_max_dist_px_;
  double corner_len_weight_;
  double corner_angle_weight_;
  double corner_dist_weight_;
  int centerline_width_{0};
  int centerline_height_{0};
  bool fps_initialized_{false};
  double fps_value_{0.0};
  std::chrono::steady_clock::time_point last_frame_tp_{};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr corner_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CornerDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
