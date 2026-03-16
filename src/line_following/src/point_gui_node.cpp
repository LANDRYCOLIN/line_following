#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

class PointGuiNode : public rclcpp::Node {
public:
  PointGuiNode() : Node("point_gui_node") {
    corner_topic_ = declare_parameter<std::string>("corner_topic", "/line/corner");
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    window_name_ = declare_parameter<std::string>("window_name", "Line Corner Simulator");
    step_px_ = declare_parameter<int>("step_px", 5);
    show_grid_ = declare_parameter<bool>("show_grid", true);
    valid_on_start_ = declare_parameter<bool>("valid_on_start", true);
    init_x_ = declare_parameter<double>("init_x", 0.5);
    init_y_ = declare_parameter<double>("init_y", 0.5);

    width_ = std::max(64, width_);
    height_ = std::max(64, height_);
    step_px_ = std::max(1, step_px_);

    {
      std::lock_guard<std::mutex> lock(point_mutex_);
      point_valid_ = valid_on_start_;
      point_px_.x = static_cast<int>(std::round(std::clamp(init_x_, 0.0, 1.0) * (width_ - 1)));
      point_px_.y = static_cast<int>(std::round(std::clamp(init_y_, 0.0, 1.0) * (height_ - 1)));
    }

    corner_pub_ = create_publisher<geometry_msgs::msg::Point>(corner_topic_, 10);

    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(window_name_, &PointGuiNode::onMouse, this);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PointGuiNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "Point GUI started. Use left drag to move, right click to clear, Q/ESC to quit.");
  }

  ~PointGuiNode() override {
    cv::destroyWindow(window_name_);
  }

private:
  static void onMouse(int event, int x, int y, int flags, void * userdata) {
    auto * self = static_cast<PointGuiNode *>(userdata);
    if (!self) {
      return;
    }
    const bool left_drag = (flags & cv::EVENT_FLAG_LBUTTON) != 0;
    if (event == cv::EVENT_LBUTTONDOWN || (event == cv::EVENT_MOUSEMOVE && left_drag)) {
      self->setPoint(x, y, true);
    } else if (event == cv::EVENT_RBUTTONDOWN) {
      self->setPoint(x, y, false);
    }
  }

  void setPoint(int x, int y, bool valid) {
    std::lock_guard<std::mutex> lock(point_mutex_);
    point_px_.x = std::clamp(x, 0, width_ - 1);
    point_px_.y = std::clamp(y, 0, height_ - 1);
    point_valid_ = valid;
  }

  void onTimer() {
    drawFrame();
    publishPoint();
    handleKeys();
  }

  void drawFrame() {
    cv::Mat canvas(height_, width_, CV_8UC3, cv::Scalar(18, 18, 22));

    if (show_grid_) {
      for (int x = 0; x < width_; x += 40) {
        cv::line(canvas, {x, 0}, {x, height_ - 1}, cv::Scalar(35, 35, 40), 1);
      }
      for (int y = 0; y < height_; y += 40) {
        cv::line(canvas, {0, y}, {width_ - 1, y}, cv::Scalar(35, 35, 40), 1);
      }
    }

    cv::Point pt;
    bool valid = false;
    {
      std::lock_guard<std::mutex> lock(point_mutex_);
      pt = point_px_;
      valid = point_valid_;
    }

    if (valid) {
      cv::circle(canvas, pt, 7, cv::Scalar(0, 180, 255), -1);
      cv::circle(canvas, pt, 14, cv::Scalar(0, 120, 200), 2);
      cv::line(canvas, {pt.x, 0}, {pt.x, height_ - 1}, cv::Scalar(60, 60, 80), 1);
      cv::line(canvas, {0, pt.y}, {width_ - 1, pt.y}, cv::Scalar(60, 60, 80), 1);
    } else {
      cv::putText(canvas, "invalid", {20, 40},
                  cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);
    }

    const double nx = valid ? (static_cast<double>(pt.x) / std::max(1, width_ - 1)) : 0.0;
    const double ny = valid ? (static_cast<double>(pt.y) / std::max(1, height_ - 1)) : 0.0;
    char buf[128];
    std::snprintf(buf, sizeof(buf), "x=%.3f y=%.3f", nx, ny);
    cv::putText(canvas, buf, {20, height_ - 20},
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 220), 2);

    cv::imshow(window_name_, canvas);
  }

  void publishPoint() {
    geometry_msgs::msg::Point msg;
    cv::Point pt;
    bool valid = false;
    {
      std::lock_guard<std::mutex> lock(point_mutex_);
      pt = point_px_;
      valid = point_valid_;
    }

    if (valid) {
      msg.x = static_cast<double>(pt.x) / std::max(1, width_ - 1);
      msg.y = static_cast<double>(pt.y) / std::max(1, height_ - 1);
      msg.z = 0.0;
    } else {
      msg.x = std::numeric_limits<double>::quiet_NaN();
      msg.y = std::numeric_limits<double>::quiet_NaN();
      msg.z = 0.0;
    }
    corner_pub_->publish(msg);
  }

  void handleKeys() {
    const int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key == 'Q') {
      rclcpp::shutdown();
      return;
    }

    int dx = 0;
    int dy = 0;
    if (key == 'w' || key == 'W') dy = -step_px_;
    if (key == 's' || key == 'S') dy = step_px_;
    if (key == 'a' || key == 'A') dx = -step_px_;
    if (key == 'd' || key == 'D') dx = step_px_;
    if (key == 'c' || key == 'C') {
      std::lock_guard<std::mutex> lock(point_mutex_);
      point_valid_ = false;
      return;
    }

    if (dx == 0 && dy == 0) {
      return;
    }

    std::lock_guard<std::mutex> lock(point_mutex_);
    point_px_.x = std::clamp(point_px_.x + dx, 0, width_ - 1);
    point_px_.y = std::clamp(point_px_.y + dy, 0, height_ - 1);
    point_valid_ = true;
  }

private:
  std::string corner_topic_;
  int width_;
  int height_;
  double publish_rate_hz_;
  std::string window_name_;
  int step_px_;
  bool show_grid_;
  bool valid_on_start_;
  double init_x_;
  double init_y_;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr corner_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex point_mutex_;
  cv::Point point_px_{0, 0};
  bool point_valid_{false};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointGuiNode>());
  rclcpp::shutdown();
  return 0;
}
