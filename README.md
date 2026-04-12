# auto_cast

一个基于 **ROS 2 Jazzy + OpenCV** 的巡线/角点感知工程，当前包含相机输入、角点检测、竖线检测、激光测距串口读取、MCU 串口桥接以及系统状态监控。

## 功能概述

当前仓库已实现：

- 相机输入节点，支持 `V4L2` 摄像头或视频文件回放
- 白色宽条角点检测，输出归一化角点坐标与横向误差
- 基于二值图的竖线检测，输出 `y=0.5` 处横坐标和相对竖直方向偏角
- 激光模块串口读取，输出距离有效位和量化距离
- 串口桥接节点，按固定协议向 MCU 下发角点、竖线、激光数据，并接收心跳/回显
- 系统监控节点，汇总视频、感知、串口、激光链路状态
- GUI 仿真模式，可直接用鼠标发布角点数据

## 系统架构

```text
/camera/image_raw
    |
    v
camera_node
    |
    v
corner_detector_node
    |- /line/error
    |- /line/corner
    |- /line/binary_image
    `- /line/debug_image
              |
              v
     vertical_line_detector_node
        |- /vertical_line/line
        |- /vertical_line/angle_deg
        |- /vertical_line/x_at_y_half
        `- /vertical_line/debug_image

laser_node
  |- /lidar_dist
  |- /lidar_valid
  |- /lidar_conf
  `- /laser/port_ok

serial_bridge_node
  |- 订阅 /line/corner /vertical_line/line /lidar_dist /lidar_valid
  `- 发布 /serial_bridge/port_ok

system_monitor_node
  `- 发布 /system_monitor/status
```

## 环境要求

- Ubuntu 24.04
- ROS 2 Jazzy
- OpenCV 系统库

安装依赖：

```bash
sudo apt update
sudo apt install \
  ros-jazzy-cv-bridge \
  ros-jazzy-rqt-image-view \
  libopencv-dev
```

## 工程结构

```text
26_auto_cast/
├── src/
│   ├── camera/
│   ├── corner_control/
│   ├── laser/
│   ├── serial_bridge/
│   ├── simulation/
│   ├── system_monitor/
│   └── vertical_line_control/
├── build/
├── install/
├── log/
└── README.md
```

## 编译

```bash
cd ~/26_auto_cast
colcon build --packages-select \
  camera corner_control laser serial_bridge simulation system_monitor vertical_line_control
source install/setup.bash
```

## 运行

主流程启动：

```bash
ros2 launch corner_control corner_control.launch.py
```

当前 `corner_control.launch.py` 会启动：

- `camera_node`
- `corner_detector_node`
- `vertical_line_detector_node`
- `laser_node`
- `serial_bridge_node`
- `system_monitor_node`
- `rqt_image_view` 仅在 `ENABLE_RQT = True` 时启动

### 默认输入源

当前 launch 默认配置是 **摄像头输入**：

```python
'use_video': False,
'device_index': 0,
'width': 640,
'height': 480,
'fps': 60,
'pixel_format': 'MJPG',
```

如果要切到视频输入，修改 [corner_control.launch.py](/home/mechax/26_auto_cast/src/corner_control/launch/corner_control.launch.py) 中相机节点参数：

```python
'use_video': True,
'video_path': '/home/mechax/26_auto_cast/test.mp4',
```

### GUI 仿真模式

```bash
ros2 launch simulation corner_sim.launch.py
```

该模式启动：

- `point_gui_node`：用鼠标/键盘发布 `/line/corner`
- `serial_bridge_node`

适合在没有相机和视觉链路时单独联调串口下发。

## 话题说明

### 输入话题

| 话题名 | 类型 | 说明 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 相机原始图像 |
| `/line/binary_image` | `sensor_msgs/msg/Image` | 供竖线检测复用的二值图 |

### 感知输出

| 话题名 | 类型 | 说明 |
| --- | --- | --- |
| `/line/error` | `std_msgs/msg/Float32` | 角点横向误差，未检测到时发布 `0.0` |
| `/line/corner` | `geometry_msgs/msg/Point` | 角点归一化坐标，未检测到时 `x/y=NaN` |
| `/line/debug_image` | `sensor_msgs/msg/Image` | 角点检测调试图 |
| `/line/binary_image` | `sensor_msgs/msg/Image` | 角点检测二值图 |
| `/vertical_line/line` | `geometry_msgs/msg/Point` | `x=x@0.5`，`y=0.5`，`z=angle_deg` |
| `/vertical_line/angle_deg` | `std_msgs/msg/Float32` | 竖线相对竖直方向偏角 |
| `/vertical_line/x_at_y_half` | `std_msgs/msg/Float32` | 竖线在 `y=0.5` 处的归一化横坐标 |
| `/vertical_line/debug_image` | `sensor_msgs/msg/Image` | 竖线检测调试图 |

### 激光与状态输出

| 话题名 | 类型 | 说明 |
| --- | --- | --- |
| `/lidar_dist` | `std_msgs/msg/UInt16` | 激光距离，单位为 `0.01 m`，范围 `0..2000` |
| `/lidar_valid` | `std_msgs/msg/UInt8` | 激光数据有效位 |
| `/lidar_conf` | `std_msgs/msg/UInt8` | 激光原始 confidence |
| `/laser/port_ok` | `std_msgs/msg/UInt8` | 激光串口是否打开 |
| `/serial_bridge/port_ok` | `std_msgs/msg/UInt8` | 桥接串口是否打开 |
| `/system_monitor/status` | `std_msgs/msg/String` | 系统状态摘要 |

## 关键默认参数

以下是当前 [corner_control.launch.py](/home/mechax/26_auto_cast/src/corner_control/launch/corner_control.launch.py) 中的默认值。

### `camera_node`

```yaml
camera_node:
  ros__parameters:
    device_index: 0
    width: 640
    height: 480
    fps: 60
    pixel_format: MJPG
    fixed_rate_output: false
    use_video: false
    video_path: /home/mechax/26_auto_cast/test.mp4
    image_topic: /camera/image_raw
```

### `corner_detector_node`

```yaml
corner_detector_node:
  ros__parameters:
    image_topic: /camera/image_raw
    threshold: 210
    auto_threshold: true
    auto_thresh_k: 0.8
    auto_thresh_min: 40
    auto_thresh_max: 235
    roi_ratio: 1.0
    morph_ksize: 5
    close_ksize: 7
    open_ksize: 7
    blur_ksize: 5
    bilateral_d: 0
    bilateral_sigma_color: 25.0
    bilateral_sigma_space: 25.0
    intersection_margin: 2
    border_margin_px: 8
    centerline_ratio: 0.45
    centerline_use_nms: true
    centerline_nms_ksize: 3
    hough_threshold: 12
    hough_min_length: 5
    hough_max_gap: 80
    corner_angle_min_deg: 60.0
    corner_angle_max_deg: 120.0
    corner_max_dist_px: 110
    corner_len_weight: 1.0
    corner_angle_weight: 0.8
    corner_dist_weight: 1.5
    binary_topic: /line/binary_image
    corner_topic: /line/corner
    publish_binary_debug: true
    publish_debug: true
    show_fps_overlay: true
```

### `vertical_line_detector_node`

```yaml
vertical_line_detector_node:
  ros__parameters:
    binary_topic: /line/binary_image
    line_topic: /vertical_line/line
    angle_topic: /vertical_line/angle_deg
    x_topic: /vertical_line/x_at_y_half
    debug_topic: /vertical_line/debug_image
    publish_debug: true
    show_fps_overlay: true
    morph_open_ksize: 3
    morph_close_ksize: 5
    border_margin_px: 8
    hough_threshold: 12
    hough_min_length: 30
    hough_max_gap: 40
    max_abs_angle_deg: 30.0
    angle_penalty: 2.0
```

### `laser_node`

```yaml
laser_node:
  ros__parameters:
    port_name: /dev/ttyACM0
    baud_rate: 230400
```

### `serial_bridge_node`

```yaml
serial_bridge_node:
  ros__parameters:
    serial_port: /dev/ttyUSB0
    corner_topic: /line/corner
    vertical_line_topic: /vertical_line/line
    laser_dist_topic: /lidar_dist
    laser_valid_topic: /lidar_valid
    send_period_ms: 20
    confidence_value: 255
    log_heartbeat: true
```

### `system_monitor_node`

```yaml
system_monitor_node:
  ros__parameters:
    image_topic: /camera/image_raw
    corner_topic: /line/corner
    vertical_line_topic: /vertical_line/line
    lidar_dist_topic: /lidar_dist
    lidar_valid_topic: /lidar_valid
    serial_port_status_topic: /serial_bridge/port_ok
    laser_port_status_topic: /laser/port_ok
    status_topic: /system_monitor/status
    publish_period_ms: 1000
    topic_timeout_ms: 500
    image_timeout_ms: 500
    port_timeout_ms: 1500
```

## 调试

查看图像：

```bash
ros2 run rqt_image_view rqt_image_view
```

查看状态：

```bash
ros2 topic echo /system_monitor/status
```

查看角点和竖线结果：

```bash
ros2 topic echo /line/corner
ros2 topic echo /vertical_line/line
```

## 串口协议

`serial_bridge_node` 下发帧格式：

```text
SOF(0xA5) | LEN | TYPE | SEQ | PAYLOAD | CRC16
```

- `TYPE=0x01` `VISION_POINT`，`LEN=6`
  `valid(1B) | x_q(2B) | y_q(2B) | conf(1B)`
- `TYPE=0x05` `VERTICAL_LINE`，`LEN=6`
  `valid(1B) | x_q(2B) | angle_q(2B) | conf(1B)`
- `TYPE=0x04` `LASER_RANGE`，`LEN=3`
  `valid(1B) | range_q(2B)`
- `TYPE=0x81` `HEARTBEAT`，`LEN=5`
- `TYPE=0x82` `VISION_ECHO`，`LEN=6`

量化方式：

- `x_q = round(clamp(norm_x, 0, 1) * 10000)`
- `y_q = round(clamp(norm_y, 0, 1) * 10000)`
- `angle_q = round(angle_deg * 100)`，按 `int16_t` 小端发送
- `range_q` 单位为 `0.01 m`

CRC 为 `CRC16-CCITT`，多项式 `0x1021`，初值 `0xFFFF`，低字节在前。

## 已知说明

- `corner_controller_node` 目前只有占位实现，主流程没有启用闭环控制
- 视觉检测默认针对明显白色宽条场景
- 光照变化较大时需要重新调整 `auto_thresh_*`、形态学和 Hough 参数
- README 说明基于 2026-04-13 仓库内代码与当前 launch 默认值整理
