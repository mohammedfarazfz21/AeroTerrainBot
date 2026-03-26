from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aeroterrabot_transform_controller',
            executable='transform_controller',
            name='transform_controller',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
