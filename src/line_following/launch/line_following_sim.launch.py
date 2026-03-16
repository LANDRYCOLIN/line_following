from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_following',
            executable='point_gui_node',
            name='point_gui_node',
            output='screen',
            parameters=[{
                'corner_topic': '/line/corner',
                'width': 640,
                'height': 480,
                'publish_rate_hz': 30.0,
                'window_name': 'Line Corner Simulator',
                'step_px': 5,
                'show_grid': True,
                'valid_on_start': True,
                'init_x': 0.5,
                'init_y': 0.5
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
        )
    ])
