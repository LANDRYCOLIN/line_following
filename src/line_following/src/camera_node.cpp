#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <string>

class CameraNode : public rclcpp::Node {
public:
  CameraNode() : Node("camera_node") {
    device_index_ = declare_parameter<int>("device_index", 0);
    width_        = declare_parameter<int>("width", 640);
    height_       = declare_parameter<int>("height", 480);
    fps_          = declare_parameter<int>("fps", 30);
    frame_id_     = declare_parameter<std::string>("frame_id", "camera_frame");
    image_topic_  = declare_parameter<std::string>("image_topic", "/camera/image_raw");

    pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS());

    // V4L2 打开摄像头
    cap_.open(device_index_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_FATAL(get_logger(), "Cannot open camera index %d", device_index_);
      throw std::runtime_error("camera open failed");
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS,          fps_);

    const int period_ms = (fps_ > 0) ? (1000 / fps_) : 33;
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&CameraNode::tick, this)
    );

    RCLCPP_INFO(get_logger(),
      "CameraNode started: index=%d, %dx%d @%dfps, topic=%s",
      device_index_, width_, height_, fps_, image_topic_.c_str());
  }

private:
  void tick() {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read frame");
      return;
    }

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id_;

    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    pub_->publish(*msg);
  }

private:
  int device_index_{0}, width_{640}, height_{480}, fps_{30};
  std::string frame_id_{"camera_frame"};
  std::string image_topic_{"/camera/image_raw"};

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
