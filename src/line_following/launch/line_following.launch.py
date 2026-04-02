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
                'device_index': 0,
                'width': 640,
                'height': 480,
                'fps': 60,
                'pixel_format': 'MJPG',
                'fixed_rate_output': False,
                'use_video': False,
                'video_path': '/home/mechax/lf_demo/test.mp4',
                'image_topic': '/camera/image_raw'
            }]
        ),

        # -------- Line Detector Node --------
        Node(
            package='line_following',
            executable='line_detector_node',
            name='line_detector_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'threshold': 210,
                'auto_threshold': True,
                'auto_thresh_k': 0.6, # 检测容忍度
                'auto_thresh_min': 20,
                'auto_thresh_max': 235,
                'roi_ratio': 1.0,
                'morph_ksize': 5,
                'close_ksize': 7,
                'open_ksize': 7,
                'blur_ksize': 5,
                'bilateral_d': 0,
                'bilateral_sigma_color': 25.0,
                'bilateral_sigma_space': 25.0,
                'intersection_margin': 2,
                'border_margin_px': 8,
                'centerline_ratio': 0.45,
                'centerline_use_nms': True,
                'centerline_nms_ksize': 3,
                'hough_threshold': 12,
                'hough_min_length': 5,
                'hough_max_gap': 80,
                'corner_angle_min_deg': 60.0,
                'corner_angle_max_deg': 120.0,
                'corner_max_dist_px': 110, # 角点与中心线的最大距离
                'corner_len_weight': 1.0,
                'corner_angle_weight': 0.8,
                'corner_dist_weight': 1.5,
                'binary_topic': '/line/binary_image',
                'corner_topic': '/line/corner',
                'publish_binary_debug': True,
                'publish_debug': True,
                'show_fps_overlay': True
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
