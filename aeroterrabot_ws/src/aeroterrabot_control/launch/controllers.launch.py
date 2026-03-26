import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch controller_manager with all AeroTerraBot controllers."""

    controllers_yaml = os.path.join(
        get_package_share_directory('aeroterrabot_control'),
        'config',
        'controllers.yaml'
    )

    # Spawn controllers sequentially: broadcaster first, then subsystem controllers
    controller_spawners = []
    controllers = [
        'joint_state_broadcaster',
        'diff_drive_controller',
        'leg_joint_position_controller',
        'transform_joint_position_controller',
        # Rotor controller starts inactive (activated on flight_mode)
    ]

    inactive_controllers = [
        'wheel_velocity_controller',
        'rotor_velocity_controller',
    ]

    for controller in controllers:
        controller_spawners.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[controller, '--controller-manager', '/controller_manager'],
                output='screen',
            )
        )

    for controller in inactive_controllers:
        controller_spawners.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    controller,
                    '--controller-manager', '/controller_manager',
                    '--inactive',
                ],
                output='screen',
            )
        )

    return LaunchDescription(controller_spawners)
