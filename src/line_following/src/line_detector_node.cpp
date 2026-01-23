#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

class LineDetectorNode : public rclcpp::Node {
public:
  LineDetectorNode() : Node("line_detector_node") {
    // -------- Parameters --------
    image_topic_   = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    error_topic_   = declare_parameter<std::string>("error_topic", "/line/error");
    debug_topic_   = declare_parameter<std::string>("debug_topic", "/line/debug_image");
    binary_topic_  = declare_parameter<std::string>("binary_topic", "/line/binary_image");
    corner_topic_  = declare_parameter<std::string>("corner_topic", "/line/corner");

    threshold_     = declare_parameter<int>("threshold", 60);
    roi_ratio_     = declare_parameter<double>("roi_ratio", 1.0);
    morph_ksize_   = declare_parameter<int>("morph_ksize", 5);
    publish_debug_ = declare_parameter<bool>("publish_debug", true);
    publish_binary_debug_ = declare_parameter<bool>("publish_binary_debug", true);

    skeleton_max_iter_    = declare_parameter<int>("skeleton_max_iter", 250);
    skeleton_smooth_ksize_= declare_parameter<int>("skeleton_smooth_ksize", 3);
    intersection_margin_  = declare_parameter<int>("intersection_margin", 2);
    border_margin_px_     = declare_parameter<int>("border_margin_px", 60);

    // -------- Publishers --------
    error_pub_ = create_publisher<std_msgs::msg::Float32>(error_topic_, 10);
    corner_pub_ = create_publisher<geometry_msgs::msg::Point>(corner_topic_, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_topic_, 1);
    binary_pub_ = create_publisher<sensor_msgs::msg::Image>(binary_topic_, 1);

    // -------- Subscriber --------
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LineDetectorNode::onImage, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(),
      "LineDetector started. image_topic=%s, error_topic=%s, corner_topic=%s, debug_topic=%s, binary_topic=%s",
      image_topic_.c_str(), error_topic_.c_str(), corner_topic_.c_str(),
      debug_topic_.c_str(), binary_topic_.c_str());
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

    // ROI：可根据参数裁剪高度
    const int roi_y = static_cast<int>(h * (1.0 - roi_ratio_));
    const int roi_h = h - roi_y;
    if (roi_y < 0 || roi_h <= 0) return;

    const cv::Rect roi_rect(0, roi_y, w, roi_h);
    cv::Mat roi = frame(roi_rect);

    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

    cv::Mat binary;
    cv::threshold(gray, binary, threshold_, 255, cv::THRESH_BINARY_INV);

    int k = std::max(1, morph_ksize_);
    if (k % 2 == 0) k += 1;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {k, k});
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    applyBorderMask(binary);

    if (publish_binary_debug_) {
      auto bin_msg = cv_bridge::CvImage(msg->header, "mono8", binary).toImageMsg();
      binary_pub_->publish(*bin_msg);
    }

    cv::Mat skeleton;
    skeleton_width_ = binary.cols;
    skeleton_height_ = binary.rows;
    const bool has_skeleton = computeSkeleton(binary.clone(), skeleton);

    std::vector<cv::Vec4f> lines;
    bool detected = false;
    if (has_skeleton) {
      detected = detectAxisLines(skeleton, lines);
    }

    cv::Point best_corner_roi;
    const bool found_corner = detected && computeLineIntersection(lines, best_corner_roi);

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

      if (has_skeleton) {
        cv::Mat vis_roi = vis(roi_rect);
        vis_roi.setTo(cv::Scalar(0, 200, 255), skeleton);
      }

      if (detected) {
        for (const auto & line : lines) {
          cv::Point2f p(line[2], line[3]);
          cv::Point2f d(line[0], line[1]);
          cv::Point2f p0 = p - 1000.0f * d;
          cv::Point2f p1 = p + 1000.0f * d;
          cv::line(vis,
                   {cvRound(p0.x), cvRound(p0.y + roi_y)},
                   {cvRound(p1.x), cvRound(p1.y + roi_y)},
                   {255, 255, 0}, 1);
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

      auto out = cv_bridge::CvImage(msg->header, "bgr8", vis).toImageMsg();
      debug_pub_->publish(*out);
    }
  }

  bool computeSkeleton(cv::Mat mask, cv::Mat &skeleton) const {
    if (mask.empty()) {
      return false;
    }
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, {3, 3});
    skeleton = cv::Mat::zeros(mask.size(), CV_8UC1);
    cv::Mat temp, eroded;

    int iterations = 0;
    while (true) {
      cv::erode(mask, eroded, element);
      cv::dilate(eroded, temp, element);
      cv::subtract(mask, temp, temp);
      cv::bitwise_or(skeleton, temp, skeleton);
      eroded.copyTo(mask);
      iterations++;
      if (cv::countNonZero(mask) == 0 || iterations >= skeleton_max_iter_) {
        break;
      }
    }
    const int smooth = std::max(3, skeleton_smooth_ksize_ | 1);
    if (smooth > 1) {
      cv::medianBlur(skeleton, skeleton, smooth);
    }
    return cv::countNonZero(skeleton) > 0;
  }

  bool detectAxisLines(const cv::Mat &skeleton, std::vector<cv::Vec4f> &lines) const {
    if (skeleton.empty()) {
      return false;
    }
    std::vector<cv::Point> points;
    cv::findNonZero(skeleton, points);
    if (points.size() < 2) {
      return false;
    }

    cv::Mat labels;
    const int clusters = std::min(2, static_cast<int>(points.size()));
    cv::Mat data(points.size(), 1, CV_32FC2);
    for (size_t i = 0; i < points.size(); ++i) {
      data.at<cv::Vec2f>(i, 0) = cv::Vec2f(points[i].x, points[i].y);
    }
    cv::kmeans(data, clusters, labels,
               cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.0),
               3, cv::KMEANS_PP_CENTERS);

    lines.clear();
    for (int cluster_idx = 0; cluster_idx < clusters; ++cluster_idx) {
      std::vector<cv::Point2f> cluster_points;
      for (int i = 0; i < labels.rows; ++i) {
        if (labels.at<int>(i) == cluster_idx) {
          cluster_points.emplace_back(points[i]);
        }
      }
      if (cluster_points.size() < 2) {
        continue;
      }
      cv::Vec4f line;
      cv::fitLine(cluster_points, line, cv::DIST_L2, 0, 0.01, 0.01);
      lines.push_back(line);
    }
    return lines.size() == 2;
  }

  bool computeLineIntersection(const std::vector<cv::Vec4f> &lines, cv::Point &intersection) const {
    if (lines.size() != 2) {
      return false;
    }
    const auto &l1 = lines[0];
    const auto &l2 = lines[1];

    cv::Point2f p1(l1[2], l1[3]);
    cv::Point2f d1(l1[0], l1[1]);
    cv::Point2f p2(l2[2], l2[3]);
    cv::Point2f d2(l2[0], l2[1]);

    float cross = d1.x * d2.y - d1.y * d2.x;
    if (std::fabs(cross) < 1e-6f) {
      return false;
    }

    cv::Point2f diff = p2 - p1;
    float t = (diff.x * d2.y - diff.y * d2.x) / cross;
    cv::Point2f pt = p1 + t * d1;
    intersection = cv::Point(cvRound(pt.x), cvRound(pt.y));

    const int min_x = std::max(0, intersection_margin_);
    const int min_y = std::max(0, intersection_margin_);
    const int max_x = std::max(min_x, skeleton_width_ - 1);
    const int max_y = std::max(min_y, skeleton_height_ - 1);
    intersection.x = std::clamp(intersection.x, min_x, max_x);
    intersection.y = std::clamp(intersection.y, min_y, max_y);
    return true;
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
  double roi_ratio_;
  int morph_ksize_;
  bool publish_debug_;
  bool publish_binary_debug_;
  int skeleton_max_iter_;
  int skeleton_smooth_ksize_;
  int intersection_margin_;
  int border_margin_px_;
  int skeleton_width_{0};
  int skeleton_height_{0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr corner_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr binary_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
