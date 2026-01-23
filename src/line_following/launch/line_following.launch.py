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
                'fps': 120,
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
                'publish_debug': True
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
