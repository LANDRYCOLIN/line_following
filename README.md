# auto_cast

一个基于 **ROS 2 (Jazzy) + OpenCV** 的巡线感知 Demo，面向 **白色宽条** 场景，支持 **视频回放或摄像头输入**，输出角点、竖线结果与误差，适合巡线小车与后续控制闭环扩展。

---

## 1. 功能概述

当前版本已实现：

- 使用 **视频或摄像头（V4L2）** 作为输入
- 自定义 **ROS 2 相机节点**，发布 `/camera/image_raw`
- 巡线检测节点（白色宽条）：
  - ROI 裁剪
  - 灰度化 + 自适应阈值（或固定阈值）+ 形态学清理
  - 距离变换提取中心线（适配宽条）
  - Hough 线段 + 角点筛选（支持 L/T 角点，非严格 90 度）
  - 角点位置归一化输出（左上角为 0,0）
- 竖向白条检测节点：
  - 复用 `/line/binary_image`
  - 只筛接近竖直方向的线段
  - 输出相对竖直方向的有符号角度偏差
  - 输出该线在 `y=0.5` 处的归一化横坐标
- 发布调试图像 `/line/debug_image`
- 串口桥接节点：按照固定帧格式将角点、竖线和激光数据下发到电控 MCU，并回读心跳
- 参数全部支持 YAML / launch 配置
- 使用 `launch` 启动

---

## 2. 系统架构

```

+----------------+
|  camera_node   |
|  (Video/V4L2)  |
+--------+-------+
     |
     |  /camera/image_raw
     v
+---------------------+
| corner_detector_node|
|  - white strip seg  |
|  - centerline/Hough |
+-----+---------+-----+
  |         | 
  |         +--> /line/debug_image (Image)
  |             /line/binary_image (Image)
  |
  +--> /line/error  (Float32)
       /line/corner (Point)
             |
             v
   +------------------------+
   | vertical_line_detector |
   +-----------+------------+
               |
               +--> /vertical_line/line (Point)
                    /vertical_line/angle_deg (Float32)
                    /vertical_line/x_at_y_half (Float32)
        |
        v
       +----------------------+
       |  serial_bridge_node  |
       |  -> UART to MCU      |
       +----------------------+

```

---

## 3. 环境要求

- Ubuntu 24.04
- ROS 2 Jazzy
- OpenCV（系统库）
- 已验证环境：
  - VMware 虚拟机 + 笔记本自带摄像头
  - `/dev/video0` 可正常访问

### 依赖安装

```bash
sudo apt update
sudo apt install \
  ros-jazzy-cv-bridge \
  ros-jazzy-rqt-image-view \
  libopencv-dev
```

---

## 4. 工程结构

以本仓库为 ROS 2 工作空间根目录（`26_auto_cast`）为例：

```
26_auto_cast/
├── src/
│   ├── camera/
│   │   ├── src/camera_node.cpp
│   │   ├── launch/camera_test.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── corner_control/
│   │   ├── src/corner_detector_node.cpp
│   │   ├── src/corner_controller_node.cpp
│   │   ├── launch/corner_control.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── simulation/
│   │   ├── src/point_gui_node.cpp
│   │   ├── launch/corner_sim.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── serial_bridge/
│   │   ├── src/serial_bridge_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── vertical_line_control/
│   │   ├── src/vertical_line_detector_node.cpp
│   │   ├── launch/vertical_line_control.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── laser/
│       ├── src/laser_node.cpp
│       ├── CMakeLists.txt
│       └── package.xml
│
├── build/                               # colcon build 生成
├── install/
├── log/
└── README.md
```

---

## 5. 编译

```bash
cd ~/26_auto_cast
colcon build --packages-select camera corner_control simulation serial_bridge laser vertical_line_control
source install/setup.bash
```

---

## 6. 运行

```bash
ros2 launch corner_control corner_control.launch.py
```

启动后将自动运行：

* 相机节点 `camera_node`
* 角点检测节点 `corner_detector_node`
* 竖线检测节点 `vertical_line_detector_node`
* 串口桥接节点 `serial_bridge_node`
* `rqt_image_view`（仅当 `ENABLE_RQT = True`）

### 6.0 输入源切换（视频 / 摄像头）

默认使用视频作为输入，可在 `corner_control.launch.py` 中调整：

```python
use_video: True
video_path: /home/mechax/26_auto_cast/test.mp4
```

如需切回摄像头，将 `use_video` 设为 `False` 并设置 `device_index`。

### 6.1 GUI 模拟器运行

```bash
ros2 launch simulation corner_sim.launch.py
```

启动后将自动运行：

* GUI 点位模拟器 `point_gui_node`（鼠标拖动设置点位，右键清除，WASD 微调，`R` 归中到 `(0.5, 0.5)`）
* 串口桥接节点 `serial_bridge_node`

---

## 7. 话题说明

### 输入话题

| 话题名                 | 类型                  | 说明     |
| ------------------- | ------------------- | ------ |
| `/camera/image_raw` | `sensor_msgs/Image` | 相机原始图像 |

### 输出话题

| 话题名                 | 类型                  | 说明               |
| ------------------- | ------------------- | ---------------- |
| `/line/error`       | `std_msgs/Float32`  | 巡线误差，范围约 [-1, 1] |
| `/line/corner`      | `geometry_msgs/Point` | 角点归一化坐标（L/T 角点，左上为 0,0） |
| `/line/binary_image`| `sensor_msgs/Image` | 二值化调试图，便于 rqt 排查 |
| `/line/debug_image` | `sensor_msgs/Image` | 巡线调试图像           |
| `/vertical_line/line` | `geometry_msgs/Point` | 竖线结果，`x=x@0.5`，`y=0.5`，`z=angle_deg` |
| `/vertical_line/angle_deg` | `std_msgs/Float32` | 竖线相对竖直方向的有符号偏差角 |
| `/vertical_line/x_at_y_half` | `std_msgs/Float32` | 竖线在 `y=0.5` 处的归一化横坐标 |

---

## 8. 参数配置

当前示例在 [src/corner_control/launch/corner_control.launch.py](src/corner_control/launch/corner_control.launch.py) 中**以内联参数**形式配置所有节点；
如果你更习惯 YAML，可以新建 `config/*.yaml` 并在 launch 中通过 `parameters=[PathJoinSubstitution([...])]` 引用。

下面给出与当前 launch 等价的 YAML 示例，便于迁移到外部配置文件。

### 8.1 角点检测节点（corner_detector_node）

```yaml
corner_detector_node:
  ros__parameters:
    image_topic: /camera/image_raw
    error_topic: /line/error
    debug_topic: /line/debug_image
    binary_topic: /line/binary_image
    corner_topic: /line/corner

    threshold: 210
    auto_threshold: true
    auto_thresh_k: 2.0
    auto_thresh_min: 160
    auto_thresh_max: 235
    roi_ratio: 1.0
    morph_ksize: 5
    close_ksize: 9
    open_ksize: 5
    blur_ksize: 5
    bilateral_d: 0
    bilateral_sigma_color: 25.0
    bilateral_sigma_space: 25.0
    intersection_margin: 2
    border_margin_px: 8        # launch 中默认 8，代码默认 60，可视场景调整
    centerline_ratio: 0.55
    centerline_use_nms: true
    centerline_nms_ksize: 3
    hough_threshold: 30
    hough_min_length: 20
    hough_max_gap: 20
    corner_angle_min_deg: 60.0
    corner_angle_max_deg: 120.0
    corner_max_dist_px: 15
    corner_len_weight: 1.0
    corner_angle_weight: 1.0
    corner_dist_weight: 1.5
    publish_binary_debug: true
    publish_debug: true
```

### 8.2 串口桥接节点（serial_bridge_node）

```yaml
serial_bridge_node:
  ros__parameters:
    serial_port: /dev/pts/4      # 示例中使用 pseudo TTY，实际 MCU 可改为 /dev/ttyUSB0
    corner_topic: /line/corner
    vertical_line_topic: /vertical_line/line
    send_period_ms: 20           # 50 Hz
    confidence_value: 255
    log_heartbeat: true
```

### 参数说明

| 参数                    | 说明                         |
| --------------------- | -------------------------- |
| `threshold`           | 白条二值化阈值                    |
| `auto_threshold`      | 是否启用自适应阈值                 |
| `auto_thresh_k`       | 自适应阈值系数（mean + k * std） |
| `auto_thresh_min`     | 自适应阈值下限                    |
| `auto_thresh_max`     | 自适应阈值上限                    |
| `roi_ratio`           | ROI 区域比例（1.0 表示整幅图像）       |
| `morph_ksize`         | 形态学开运算核大小                 |
| `close_ksize`         | 形态学闭运算核大小（连通白条）         |
| `open_ksize`          | 形态学开运算核大小（去噪）           |
| `blur_ksize`          | 阈值前的 Gaussian 模糊核大小（奇数）  |
| `bilateral_d`         | 双边滤波采样直径（0 则关闭）         |
| `bilateral_sigma_color` | 双边滤波颜色参数               |
| `bilateral_sigma_space` | 双边滤波空间参数               |
| `intersection_margin` | 交点坐标的安全边界（像素）             |
| `border_margin_px`   | ROI 内部需忽略的边缘宽度               |
| `centerline_ratio`    | 中心线阈值比例（距离变换占比）        |
| `centerline_use_nms`  | 是否启用中心线 NMS 提取            |
| `centerline_nms_ksize`| 中心线 NMS 核大小               |
| `hough_threshold`     | Hough 线段检测阈值               |
| `hough_min_length`    | Hough 最小线段长度              |
| `hough_max_gap`       | Hough 线段最大间隔              |
| `corner_angle_min_deg`| 角点最小夹角（度）                 |
| `corner_angle_max_deg`| 角点最大夹角（度）                 |
| `corner_max_dist_px`  | 交点到线段的最大距离（像素）           |
| `corner_len_weight`   | 线段长度评分权重                  |
| `corner_angle_weight` | 角度评分权重                    |
| `corner_dist_weight`  | 距离评分权重                    |
| `publish_binary_debug`| 是否发布二值化调试图                 |
| `publish_debug`       | 是否发布叠加调试图                  |
| `serial_port`         | 串口设备路径（如 `/dev/ttyUSB0`）       |
| `vertical_line_topic` | 竖线结果输入话题                    |
| `send_period_ms`      | 串口下发周期（毫秒）                |
| `confidence_value`    | 无额外置信度时的填充值               |
| `log_heartbeat`       | 是否打印 MCU 心跳日志（DEBUG 级别）      |

---

## 9. 调试与可视化

### 查看调试图像

```bash
ros2 run rqt_image_view rqt_image_view
ros2 topic hz /line/corner #查看实时发布帧率 
```

选择话题：

```
/line/debug_image
```

### 查看巡线误差

```bash
ros2 topic echo /line/error
```

---

## 10. 已知说明

* 仅实现 **感知部分**，未包含控制器（PID / cmd_vel）
* 当前假设地面存在 **明显白色宽条**
* 光照变化较大时需调整 `auto_thresh_*` 或 `threshold`
* 镜头畸变较大时建议使用 `camera_calibration` 先标定，再在相机节点中进行去畸变以提升识别精度
* 串口帧格式：`SOF(0xA5) | LEN | TYPE | SEQ | PAYLOAD | CRC16`  
  * VISION_POINT (`TYPE=0x01`, LEN=6)：`valid(1B) | x_q(2B) | y_q(2B) | conf(1B)`，`x_q=y_q=round(norm*10000)`  
  * VERTICAL_LINE (`TYPE=0x05`, LEN=6)：`valid(1B) | x_q(2B) | angle_q(2B) | conf(1B)`  
    * `x_q = round(clamp(x@0.5, 0, 1) * 10000)`
    * `angle_q = round(angle_deg * 100)`，`int16_t` 小端发送
  * MCU_HEARTBEAT (`TYPE=0x81`, LEN=5)：`mode | err | seq_echo | counter(2B)`  
  * CRC16-CCITT，多项式 `0x1021`，初值 `0xFFFF`，不反转，小端发送（低字节在前）

---

## 11. 后续可扩展方向

* 添加 `corner_controller_node`（误差 → 速度控制）
* PID 参数配置化
* 自适应阈值 / 光照鲁棒性增强
* 接入实体小车或仿真环境
* 使用深度学习替代传统视觉
