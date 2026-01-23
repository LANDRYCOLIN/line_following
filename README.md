# line_following

一个基于 **ROS 2 (Jazzy) + OpenCV** 的巡线感知 Demo，实现了从 **真实摄像头采集 → 黑线识别 → 误差输出** 的完整视觉链路，适合用于巡线小车以及后续控制闭环扩展。

---

## 1. 功能概述

当前版本已实现：

- 使用 **笔记本摄像头（V4L2）** 作为输入
- 自定义 **ROS 2 相机节点**，发布 `/camera/image_raw`
- 巡线检测节点：
  - 全图 ROI 处理（默认使用整幅图像）
  - 灰度化 + 二值化 + 形态学去噪
  - 骨架提取（skeletonization）+ 直线拟合，适配有宽度的黑胶带
  - 直角交点检测（两条黑线交汇点）
  - 质心位置归一化输出（左上角为 0,0）
- 发布调试图像 `/line/debug_image`
- 串口桥接节点：按照固定帧格式将视觉坐标以 50 Hz 下发到电控 MCU，并回读心跳
- 参数全部支持 YAML / launch 配置
- 使用 `launch` 启动

---

## 2. 系统架构

```

+----------------+
|  camera_node   |
|  (OpenCV/V4L2) |
+--------+-------+
     |
     |  /camera/image_raw
     v
+---------------------+
| line_detector_node  |
|  - black line detect|
|  - corner extract   |
+-----+---------+-----+
  |         |
  |         +--> /line/debug_image (Image)
  |             /line/binary_image (Image)
  |
  +--> /line/error  (Float32)
       /line/corner (Point)
        |
        v
       +----------------------+
       |  serial_bridge_node  |
       |  -> UART to MCU      |
       +----------------------+

````

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
````

---

## 4. 工程结构

以本仓库为 ROS 2 工作空间根目录（lf_demo）为例：

```
lf_demo/
├── src/
│   └── line_following/
│       ├── src/
│       │   ├── camera_node.cpp          # 相机发布节点
│       │   ├── line_detector_node.cpp   # 巡线检测节点（黑线+直角交点）
│       │   ├── line_controller_node.cpp # 预留控制器节点（当前为占位实现）
│       │   └── serial_bridge_node.cpp   # 串口桥接到 MCU
│       │
│       ├── launch/
│       │   └── line_following.launch.py # 一键启动全流程
│       │
│       ├── include/                     # 头文件（如需拆分）
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
cd ~/lf_demo
colcon build --packages-select line_following
source install/setup.bash
```

---

## 6. 运行

```bash
ros2 launch line_following line_following.launch.py
```

启动后将自动运行：

* 相机节点 `camera_node`
* 巡线检测节点 `line_detector_node`
* 串口桥接节点 `serial_bridge_node`
* `rqt_image_view`

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
| `/line/corner`      | `geometry_msgs/Point` | 直角交点归一化坐标（左上为 0,0） |
| `/line/binary_image`| `sensor_msgs/Image` | 二值化调试图，便于 rqt 排查 |
| `/line/debug_image` | `sensor_msgs/Image` | 巡线调试图像           |

---

## 8. 参数配置

当前示例在 [src/line_following/launch/line_following.launch.py](src/line_following/launch/line_following.launch.py) 中**以内联参数**形式配置所有节点；
如果你更习惯 YAML，可以新建 `config/*.yaml` 并在 launch 中通过 `parameters=[PathJoinSubstitution([...])]` 引用。

下面给出与当前 launch 等价的 YAML 示例，便于迁移到外部配置文件。

### 8.1 巡线检测节点（line_detector_node）

```yaml
line_detector_node:
  ros__parameters:
    image_topic: /camera/image_raw
    error_topic: /line/error
    debug_topic: /line/debug_image
    binary_topic: /line/binary_image
    corner_topic: /line/corner

    threshold: 60
    roi_ratio: 1.0
    morph_ksize: 5
    blur_ksize: 5
    bilateral_d: 0
    bilateral_sigma_color: 25.0
    bilateral_sigma_space: 25.0
    skeleton_max_iter: 250
    skeleton_smooth_ksize: 3
    intersection_margin: 2
    border_margin_px: 8        # launch 中默认 8，代码默认 60，可视场景调整
    publish_binary_debug: true
    publish_debug: true
```

### 8.2 串口桥接节点（serial_bridge_node）

```yaml
serial_bridge_node:
  ros__parameters:
    serial_port: /dev/pts/4      # 示例中使用 pseudo TTY，实际 MCU 可改为 /dev/ttyUSB0
    corner_topic: /line/corner
    send_period_ms: 20           # 50 Hz
    confidence_value: 255
    log_heartbeat: true
```

### 参数说明

| 参数                    | 说明                         |
| --------------------- | -------------------------- |
| `threshold`           | 黑线二值化阈值                    |
| `roi_ratio`           | ROI 区域比例（1.0 表示整幅图像）       |
| `morph_ksize`         | 形态学开运算核大小                 |
| `blur_ksize`          | 阈值前的 Gaussian 模糊核大小（奇数）  |
| `bilateral_d`         | 双边滤波采样直径（0 则关闭）         |
| `bilateral_sigma_color` | 双边滤波颜色参数               |
| `bilateral_sigma_space` | 双边滤波空间参数               |
| `skeleton_max_iter`   | 骨架提取的最大迭代次数               |
| `skeleton_smooth_ksize` | 骨架图的平滑核尺寸（需为奇数）         |
| `intersection_margin` | 交点坐标的安全边界（像素）             |
| `border_margin_px`   | ROI 内部需忽略的边缘宽度               |
| `publish_binary_debug`| 是否发布二值化调试图                 |
| `publish_debug`       | 是否发布叠加调试图                  |
| `serial_port`         | 串口设备路径（如 `/dev/ttyUSB0`）       |
| `send_period_ms`      | 串口下发周期（毫秒）                |
| `confidence_value`    | 无额外置信度时的填充值               |
| `log_heartbeat`       | 是否打印 MCU 心跳日志（DEBUG 级别）      |

---

## 9. 调试与可视化

### 查看调试图像

```bash
ros2 run rqt_image_view rqt_image_view
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
* 当前假设地面存在 **明显黑色线条**
* 光照变化较大时需调整 `threshold` 或引入自适应算法
* 镜头畸变较大时建议使用 `camera_calibration` 先标定，再在相机节点中进行去畸变以提升识别精度
* 串口帧格式：`SOF(0xA5) | LEN | TYPE | SEQ | PAYLOAD | CRC16`  
  * VISION_POINT (`TYPE=0x01`, LEN=6)：`valid(1B) | x_q(2B) | y_q(2B) | conf(1B)`，`x_q=y_q=round(norm*10000)`  
  * MCU_HEARTBEAT (`TYPE=0x81`, LEN=5)：`mode | err | seq_echo | counter(2B)`  
  * CRC16-CCITT，多项式 `0x1021`，初值 `0xFFFF`，不反转，小端发送（低字节在前）

---

## 11. 后续可扩展方向

* 添加 `line_controller_node`（误差 → 速度控制）
* PID 参数配置化
* 自适应阈值 / 光照鲁棒性增强
* 接入实体小车或仿真环境
* 使用深度学习替代传统视觉
