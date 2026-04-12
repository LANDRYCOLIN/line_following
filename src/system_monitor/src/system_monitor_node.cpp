#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

class SystemMonitorNode : public rclcpp::Node {
public:
  SystemMonitorNode() : Node("system_monitor_node") {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    corner_topic_ = declare_parameter<std::string>("corner_topic", "/line/corner");
    vertical_line_topic_ = declare_parameter<std::string>("vertical_line_topic", "/vertical_line/line");
    lidar_dist_topic_ = declare_parameter<std::string>("lidar_dist_topic", "/lidar_dist");
    lidar_valid_topic_ = declare_parameter<std::string>("lidar_valid_topic", "/lidar_valid");
    serial_port_status_topic_ = declare_parameter<std::string>("serial_port_status_topic", "/serial_bridge/port_ok");
    laser_port_status_topic_ = declare_parameter<std::string>("laser_port_status_topic", "/laser/port_ok");
    status_topic_ = declare_parameter<std::string>("status_topic", "/system_monitor/status");

    print_period_ms_ = declare_parameter<int>("publish_period_ms", 1000);
    topic_timeout_ms_ = declare_parameter<int>("topic_timeout_ms", 500);
    image_timeout_ms_ = declare_parameter<int>("image_timeout_ms", 500);
    port_timeout_ms_ = declare_parameter<int>("port_timeout_ms", 1500);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SystemMonitorNode::onImage, this, std::placeholders::_1));
    corner_sub_ = create_subscription<geometry_msgs::msg::Point>(
      corner_topic_, 10,
      std::bind(&SystemMonitorNode::onCorner, this, std::placeholders::_1));
    vertical_line_sub_ = create_subscription<geometry_msgs::msg::Point>(
      vertical_line_topic_, 10,
      std::bind(&SystemMonitorNode::onVerticalLine, this, std::placeholders::_1));
    lidar_dist_sub_ = create_subscription<std_msgs::msg::UInt16>(
      lidar_dist_topic_, 10,
      std::bind(&SystemMonitorNode::onLidarDist, this, std::placeholders::_1));
    lidar_valid_sub_ = create_subscription<std_msgs::msg::UInt8>(
      lidar_valid_topic_, 10,
      std::bind(&SystemMonitorNode::onLidarValid, this, std::placeholders::_1));
    serial_port_status_sub_ = create_subscription<std_msgs::msg::UInt8>(
      serial_port_status_topic_, 10,
      std::bind(&SystemMonitorNode::onSerialPortStatus, this, std::placeholders::_1));
    laser_port_status_sub_ = create_subscription<std_msgs::msg::UInt8>(
      laser_port_status_topic_, 10,
      std::bind(&SystemMonitorNode::onLaserPortStatus, this, std::placeholders::_1));
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(100, print_period_ms_)),
      std::bind(&SystemMonitorNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "System monitor started");
  }

private:
  struct TopicState {
    rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time window_start{0, 0, RCL_ROS_TIME};
    int count_in_window{0};
    double hz{0.0};
    bool seen{false};
    bool valid{false};
  };

  struct PortState {
    rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
    bool seen{false};
    bool ok{false};
  };

  void touchTopic(TopicState & state, bool valid) {
    const auto now = this->now();
    if (!state.seen) {
      state.window_start = now;
      state.hz = 0.0;
    } else {
      const double dt = (now - state.window_start).seconds();
      if (dt >= 1.0) {
        state.hz = static_cast<double>(state.count_in_window) / dt;
        state.count_in_window = 0;
        state.window_start = now;
      }
    }
    state.last_stamp = now;
    state.count_in_window += 1;
    state.seen = true;
    state.valid = valid;
  }

  void touchPort(PortState & state, bool ok) {
    state.last_stamp = this->now();
    state.seen = true;
    state.ok = ok;
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr) {
    touchTopic(image_state_, true);
  }

  void onCorner(const geometry_msgs::msg::Point::SharedPtr msg) {
    const bool valid = std::isfinite(msg->x) && std::isfinite(msg->y);
    touchTopic(corner_state_, valid);
  }

  void onVerticalLine(const geometry_msgs::msg::Point::SharedPtr msg) {
    const bool valid = std::isfinite(msg->x) && std::isfinite(msg->z);
    touchTopic(vertical_line_state_, valid);
  }

  void onLidarDist(const std_msgs::msg::UInt16::SharedPtr msg) {
    latest_lidar_dist_ = msg->data;
    const bool valid = latest_lidar_valid_ && msg->data > 0;
    touchTopic(lidar_state_, valid);
  }

  void onLidarValid(const std_msgs::msg::UInt8::SharedPtr msg) {
    latest_lidar_valid_ = (msg->data != 0);
  }

  void onSerialPortStatus(const std_msgs::msg::UInt8::SharedPtr msg) {
    touchPort(serial_port_state_, msg->data != 0);
  }

  void onLaserPortStatus(const std_msgs::msg::UInt8::SharedPtr msg) {
    touchPort(laser_port_state_, msg->data != 0);
  }

  bool isFresh(const rclcpp::Time & stamp, int timeout_ms) const {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= static_cast<double>(timeout_ms) / 1000.0;
  }

  std::string formatTopic(const std::string & name,
                          const TopicState & state,
                          int timeout_ms) const {
    std::ostringstream oss;
    oss << name << ":";
    if (!state.seen || !isFresh(state.last_stamp, timeout_ms)) {
      oss << "DOWN";
      return oss.str();
    }
    oss << (state.valid ? "OK" : "MISS")
        << " " << std::fixed << std::setprecision(1) << state.hz << "Hz";
    return oss.str();
  }

  std::string formatPort(const std::string & name, const PortState & state) const {
    std::ostringstream oss;
    oss << name << ":";
    if (!state.seen || !isFresh(state.last_stamp, port_timeout_ms_)) {
      oss << "DOWN";
      return oss.str();
    }
    oss << (state.ok ? "OK" : "ERR");
    return oss.str();
  }

  void onTimer() {
    std::ostringstream oss;
    oss
      << formatTopic("video", image_state_, image_timeout_ms_) << " | "
      << formatTopic("corner", corner_state_, topic_timeout_ms_) << " | "
      << formatTopic("vline", vertical_line_state_, topic_timeout_ms_) << " | "
      << formatTopic("lidar", lidar_state_, topic_timeout_ms_) << " | "
      << formatPort("ttyUSB", serial_port_state_) << " | "
      << formatPort("ttyACM", laser_port_state_);

    if (lidar_state_.seen && isFresh(lidar_state_.last_stamp, topic_timeout_ms_)) {
      oss << " | lidar_q=" << latest_lidar_dist_
          << " valid=" << (latest_lidar_valid_ ? 1 : 0);
    }

    std_msgs::msg::String status_msg;
    status_msg.data = oss.str();
    status_pub_->publish(status_msg);
  }

  std::string image_topic_;
  std::string corner_topic_;
  std::string vertical_line_topic_;
  std::string lidar_dist_topic_;
  std::string lidar_valid_topic_;
  std::string serial_port_status_topic_;
  std::string laser_port_status_topic_;
  std::string status_topic_;

  int print_period_ms_;
  int topic_timeout_ms_;
  int image_timeout_ms_;
  int port_timeout_ms_;

  bool latest_lidar_valid_{false};
  uint16_t latest_lidar_dist_{0};

  TopicState image_state_;
  TopicState corner_state_;
  TopicState vertical_line_state_;
  TopicState lidar_state_;
  PortState serial_port_state_;
  PortState laser_port_state_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr corner_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr vertical_line_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr lidar_dist_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr lidar_valid_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr serial_port_status_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr laser_port_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
