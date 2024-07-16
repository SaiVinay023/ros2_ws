from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bt_bumpgoo',
            executable='bt_bumpgo_main',
            name='bt_bumpgo_main',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('input_scan', '/scan_raw'),
                ('output_vel', '/cmd_vel')
            ]
        ),
        Node(
            package='monitor',
            executable='monitor_0',
            name='monitor_0',
            output='screen'
        ),
    ])
