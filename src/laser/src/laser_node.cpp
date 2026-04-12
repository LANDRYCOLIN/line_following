#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <vector>
#include <cstring>
#include <cstdint>
#include <cmath>

/* 激光模块数据结构 */
struct __attribute__((packed)) LidarPoint
{
    uint16_t distance;
    uint16_t noise;
    uint32_t peak;
    uint8_t confidence;
    uint32_t intg;
    int16_t reftof;
};

class LaserNode : public rclcpp::Node
{
public:
    LaserNode() : Node("laser_node")
    {
        // 按照协议，range_q 范围 [0, 2000]，单位 0.01m
        dist_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/lidar_dist", 10);
        // 为了方便 serial_bridge 获取有效位，额外发布 valid
        valid_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/lidar_valid", 10);
        // 保留 conf_pub_
        conf_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/lidar_conf", 10);
        port_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/laser/port_ok", 10);

        port_name_ = this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
        baud_rate_ = this->declare_parameter<int>("baud_rate", 230400);

        fd_ = open_serial(port_name_.c_str(), baud_rate_);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open lidar port: %s", port_name_.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Laser node started on %s at %d baud", port_name_.c_str(), baud_rate_);

        last_rx_time_ = this->now();

        // 高频读取串口数据（2ms）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&LaserNode::read_loop, this));

        // 50Hz (20ms) 按照协议频率定频发布数据帧给 serial_bridge
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&LaserNode::publish_loop, this));
    }

    ~LaserNode()
    {
        if (fd_ >= 0) close(fd_);
    }

private:
    int open_serial(const char *port, int baud)
    {
        int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) return -1;

        termios tty{};
        tcgetattr(fd, &tty);

        cfmakeraw(&tty);
        
        speed_t speed;
        switch(baud) {
            case 9600: speed = B9600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            default: speed = B230400; break;
        }

        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        tcsetattr(fd, TCSANOW, &tty);

        return fd;
    }

    bool parse_lidar(std::vector<uint8_t> &buffer, uint16_t &dist, uint8_t &conf)
    {
        while(buffer.size() >= 10)
        {
            bool header_found = false;
            size_t header_pos = 0;

            for(size_t i=0; i<buffer.size()-3; i++)
            {
                if(buffer[i]==0xAA &&
                   buffer[i+1]==0xAA &&
                   buffer[i+2]==0xAA &&
                   buffer[i+3]==0xAA)
                {
                    header_found = true;
                    header_pos = i;
                    break;
                }
            }

            if(!header_found)
            {
                buffer.clear();
                return false;
            }

            if(header_pos > 0)
            {
                buffer.erase(buffer.begin(), buffer.begin() + header_pos);
                continue;
            }

            uint16_t payload_len = buffer[8] | (buffer[9]<<8);
            size_t total_len = 10 + payload_len + 1;

            if(buffer.size() < total_len)
                return false;

            LidarPoint point;
            memcpy(&point, &buffer[10], sizeof(LidarPoint));

            dist = point.distance;
            conf = point.confidence;

            buffer.erase(buffer.begin(), buffer.begin() + total_len);

            return true;
        }

        return false;
    }

    void read_loop()
    {
        if (fd_ < 0) return;

        uint8_t temp[512];
        int n = read(fd_, temp, sizeof(temp));

        if(n > 0)
            buffer_.insert(buffer_.end(), temp, temp + n);

        uint16_t dist = 0;
        uint8_t conf = 0;

        // Parse all complete frames in the buffer
        while(parse_lidar(buffer_, dist, conf))
        {
            latest_dist_mm_ = dist;
            latest_conf_ = conf;
            last_rx_time_ = this->now();
        }
    }

    void publish_loop()
    {
        auto now = this->now();
        double dt = (now - last_rx_time_).seconds();

        uint8_t valid = 1;
        uint16_t range_q = 0;

        // 协议要求：>200ms 未收到有效帧判定超时 (激光数据无效)
        if (dt > 0.200) {
            valid = 0;
        } else if (latest_dist_mm_ == 0 || latest_dist_mm_ > 20000) {
            // "range_m ∈ [0,20.00]" 超过 20 米则无效
            valid = 0;
        } else {
            // 正常数据转换
            // 原数据单位是 mm，转换为协议所需的单位 0.01m (厘米)
            // 即 range_q = round( (dist_mm / 1000.0) * 100.0 ) = round(dist_mm / 10.0)
            range_q = static_cast<uint16_t>(std::round(latest_dist_mm_ / 10.0));
            
            // 钳位
            if (range_q > 2000) {
                range_q = 2000; 
            }
        }

        // 当 valid=0 时，按协议 range_q 填 0
        if (valid == 0) {
            range_q = 0;
        }

        // 1. 发布 range_q (对应协议中的2字节距离字段)
        std_msgs::msg::UInt16 dist_msg;
        dist_msg.data = range_q;
        dist_pub_->publish(dist_msg);

        // 2. 发布 valid 标志 (供serial_bridge_node使用)
        std_msgs::msg::UInt8 valid_msg;
        valid_msg.data = valid;
        valid_pub_->publish(valid_msg);

        // 3. 发布 conf 标志 
        std_msgs::msg::UInt8 conf_msg;
        conf_msg.data = latest_conf_;
        conf_pub_->publish(conf_msg);

        std_msgs::msg::UInt8 port_msg;
        port_msg.data = (fd_ >= 0) ? 1 : 0;
        port_status_pub_->publish(port_msg);
    }

    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr dist_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr valid_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr conf_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr port_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    
    std::string port_name_;
    int baud_rate_;
    int fd_{-1};
    std::vector<uint8_t> buffer_;

    uint16_t latest_dist_mm_{0};
    uint8_t latest_conf_{0};
    rclcpp::Time last_rx_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserNode>());
    rclcpp::shutdown();
    return 0;
}
