from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # -------- Camera Node --------
        Node(
            package='line_following',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device_index': 0,                              # 摄像头设备编号
                'width': 640,                                  # 请求采集宽度
                'height': 480,                                 # 请求采集高度
                'fps': 60,                                     # 请求采集帧率
                'pixel_format': 'MJPG',                        # V4L2 请求像素格式
                'fixed_rate_output': False,                    # 是否按固定定时器发布
                'use_video': True,                             # True: 读取视频文件; False: 读取摄像头
                'video_path': '/home/mechax/lf_demo/test.mp4', # 视频输入路径
                'save_video': False,                           # 是否将当前输入另存为视频
                'save_video_path': '/home/mechax/lf_demo/output/camera_record.mp4',  # 保存视频路径
                'save_video_fourcc': 'MJPG',                  # 保存视频编码 fourcc
                'image_topic': '/camera/image_raw'             # 发布图像话题
            }]
        ),

        # -------- Line Detector Node --------
        Node(
            package='line_following',
            executable='line_detector_node',
            name='line_detector_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',      # 输入图像话题
                'threshold': 210,                        # 固定阈值模式下的二值化阈值
                'auto_threshold': True,                  # 是否启用自动阈值
                'auto_thresh_k': 1.4,                   # 自动阈值灵敏度/检测容忍度
                'auto_thresh_min': 160,                 # 自动阈值下限
                'auto_thresh_max': 235,                 # 自动阈值上限
                'roi_ratio': 1.0,                       # ROI 高度占整幅图像的比例
                'morph_ksize': 5,                       # 基础形态学处理核大小
                'close_ksize': 9,                       # 闭运算核大小，用于连接断裂区域
                'open_ksize': 5,                        # 开运算核大小，用于去除小噪点
                'blur_ksize': 5,                        # 高斯模糊核大小
                'bilateral_d': 0,                       # 双边滤波邻域直径，0 表示关闭
                'bilateral_sigma_color': 25.0,          # 双边滤波颜色域 sigma
                'bilateral_sigma_space': 25.0,          # 双边滤波空间域 sigma
                'intersection_margin': 2,               # 交点结果与边界保持的最小裕量
                'border_margin_px': 8,                  # 屏蔽图像边缘干扰的边界宽度
                'centerline_ratio': 0.45,               # 距离变换提取中心线时的阈值比例
                'centerline_use_nms': True,             # 中心线提取时是否启用非极大值抑制
                'centerline_nms_ksize': 3,              # 中心线 NMS 核大小
                'hough_threshold': 30,                  # Hough 直线检测阈值
                'hough_min_length': 20,                 # Hough 最短线段长度
                'hough_max_gap': 20,                    # Hough 允许的最大断裂间隔
                'corner_angle_min_deg': 60.0,           # 角点夹角下限
                'corner_angle_max_deg': 120.0,          # 角点夹角上限
                'corner_max_dist_px': 70,               # 角点与中心线的最大距离
                'corner_len_weight': 1.0,               # 角点评分中的线段长度权重
                'corner_angle_weight': 1.0,             # 角点评分中的角度偏差权重
                'corner_dist_weight': 1.5,              # 角点评分中的距离惩罚权重
                'binary_topic': '/line/binary_image',   # 二值化调试图像输出话题
                'corner_topic': '/line/corner',         # 角点输出话题
                'publish_binary_debug': False,          # 是否发布二值图调试画面
                'publish_debug': True,                  # 是否发布检测结果调试画面
                'show_fps_overlay': True,               # 调试图上是否显示 FPS
                'save_output_video': True,              # 是否保存处理后的输出视频
                'output_video_path': '/home/mechax/lf_demo/output/line_detector_output.mp4',  # 输出视频路径
                'output_video_fourcc': 'MJPG',          # 输出视频编码 fourcc
                'output_video_fps': 30.0                # 输出视频保存帧率
            }]
        ),

        Node(
            package='line_following',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'corner_topic': '/line/corner',
                'send_period_ms': 20,
                'confidence_value': 255,
                'log_heartbeat': True
            }]
        ),

        # -------- rqt_image_view (optional) --------
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen'
        )
    ])
