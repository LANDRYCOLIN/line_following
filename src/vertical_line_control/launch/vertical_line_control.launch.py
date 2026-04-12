from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
        )
    ])
