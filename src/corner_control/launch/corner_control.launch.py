from launch import LaunchDescription
from launch_ros.actions import Node

ENABLE_RQT = False

def generate_launch_description():
    nodes = [

        # -------- Camera Node --------
        Node(
            package='camera',
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
                'video_path': '/home/mechax/26_auto_cast/test.mp4',
                'image_topic': '/camera/image_raw'
            }]
        ),

        # -------- Corner Detector Node --------
        Node(
            package='corner_control',
            executable='corner_detector_node',
            name='corner_detector_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'threshold': 210,
                'auto_threshold': True,
                'auto_thresh_k': 0.8, # 检测容忍度
                'auto_thresh_min': 40,
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

        # -------- Vertical Line Detector Node --------
        Node(
            package='vertical_line_control',
            executable='vertical_line_detector_node',
            name='vertical_line_detector_node',
            output='screen',
            parameters=[{
                'binary_topic': '/line/binary_image',
                'line_topic': '/vertical_line/line',
                'angle_topic': '/vertical_line/angle_deg',
                'x_topic': '/vertical_line/x_at_y_half',
                'debug_topic': '/vertical_line/debug_image',
                'publish_debug': True,
                'show_fps_overlay': True,
                'morph_open_ksize': 3,
                'morph_close_ksize': 5,
                'border_margin_px': 8,
                'hough_threshold': 12,
                'hough_min_length': 30,
                'hough_max_gap': 40,
                'max_abs_angle_deg': 30.0,
                'angle_penalty': 2.0
            }]
        ),

        # -------- Laser Node --------
        Node(
            package='laser',
            executable='laser_node',
            name='laser_node',
            output='screen',
            parameters=[{
                'port_name': '/dev/ttyACM0',
                'baud_rate': 230400
            }]
        ),

        Node(
            package='serial_bridge',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'corner_topic': '/line/corner',
                'laser_dist_topic': '/lidar_dist',
                'laser_valid_topic': '/lidar_valid',
                'send_period_ms': 20,
                'confidence_value': 255,
                'log_heartbeat': True
            }]
        ),

        Node(
            package='system_monitor',
            executable='system_monitor_node',
            name='system_monitor_node',
            output='log',
            parameters=[{
                'image_topic': '/camera/image_raw',
                'corner_topic': '/line/corner',
                'vertical_line_topic': '/vertical_line/line',
                'lidar_dist_topic': '/lidar_dist',
                'lidar_valid_topic': '/lidar_valid',
                'serial_port_status_topic': '/serial_bridge/port_ok',
                'laser_port_status_topic': '/laser/port_ok',
                'status_topic': '/system_monitor/status',
                'publish_period_ms': 1000,
                'topic_timeout_ms': 500,
                'image_timeout_ms': 500,
                'port_timeout_ms': 1500
            }]
        ),
    ]

    if ENABLE_RQT:
        nodes.append(
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                name='rqt_image_view',
                output='screen'
            )
        )

    return LaunchDescription(nodes)
