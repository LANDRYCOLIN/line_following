from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_index = LaunchConfiguration('device_index')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps = LaunchConfiguration('fps')
    image_topic = LaunchConfiguration('image_topic')
    pixel_format = LaunchConfiguration('pixel_format')

    return LaunchDescription([
        DeclareLaunchArgument('device_index', default_value='0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='60'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('pixel_format', default_value='MJPG'),

        Node(
            package='camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device_index': device_index,
                'width': width,
                'height': height,
                'fps': fps,
                'pixel_format': pixel_format,
                'fixed_rate_output': False,
                'use_video': False,
                'image_topic': image_topic
            }]
        ),
    ])
