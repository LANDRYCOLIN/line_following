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
- 参数全部支持 YAML / launch 配置
- 使用 `launch` 启动

---

## 2. 系统架构

```

+----------------+
|  camera_node   |
|  (OpenCV)      |
+--------+-------+
|
|  /camera/image_raw
v
+---------------------+
| line_detector_node  |
|  - black line detect|
|  - error compute    |
+--------+------------+
|
|  /line/error (Float32)
|
+--> /line/debug_image (Image)

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

```
line_following/
├── src/
│   ├── camera_node.cpp          # 相机发布节点
│   ├── line_detector_node.cpp   # 巡线检测节点
│
├── launch/
│   └── line_following_full.launch.py
│
├── config/
│   └── line_detector.yaml       # 巡线参数配置
│
├── CMakeLists.txt
├── package.xml
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

## 8. 参数配置（config/line_detector.yaml）

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
    skeleton_max_iter: 250
    skeleton_smooth_ksize: 3
    intersection_margin: 2
    border_margin_px: 8
    publish_binary_debug: true
    publish_debug: true
```

### 参数说明

| 参数                    | 说明                         |
| --------------------- | -------------------------- |
| `threshold`           | 黑线二值化阈值                    |
| `roi_ratio`           | ROI 区域比例（1.0 表示整幅图像）       |
| `morph_ksize`         | 形态学开运算核大小                 |
| `skeleton_max_iter`   | 骨架提取的最大迭代次数               |
| `skeleton_smooth_ksize` | 骨架图的平滑核尺寸（需为奇数）         |
| `intersection_margin` | 交点坐标的安全边界（像素）             |
| `border_margin_px`   | ROI 内部需忽略的边缘宽度               |
| `publish_binary_debug`| 是否发布二值化调试图                 |
| `publish_debug`       | 是否发布叠加调试图                  |

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

---

## 11. 后续可扩展方向

* 添加 `line_controller_node`（误差 → 速度控制）
* PID 参数配置化
* 自适应阈值 / 光照鲁棒性增强
* 接入实体小车或仿真环境
* 使用深度学习替代传统视觉
