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
                'fps': 30,
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
                'threshold': 60,
                'roi_ratio': 1.0,
                'morph_ksize': 5,
                'blur_ksize': 5,
                'bilateral_d': 0,
                'bilateral_sigma_color': 25.0,
                'bilateral_sigma_space': 25.0,
                'skeleton_max_iter': 250,
                'skeleton_smooth_ksize': 3,
                'intersection_margin': 2,
                'border_margin_px': 8,
                'binary_topic': '/line/binary_image',
                'corner_topic': '/line/corner',
                'publish_binary_debug': True,
                'publish_debug': True
            }]
        ),

        Node(
            package='line_following',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/pts/4',
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
