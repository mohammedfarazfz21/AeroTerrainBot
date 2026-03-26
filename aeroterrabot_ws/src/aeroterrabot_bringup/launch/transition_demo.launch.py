import os
import time
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('aeroterrabot_gazebo')
    pkg_transform = get_package_share_directory('aeroterrabot_transform_controller')

    # Simulation environment & Robot
    # Start Gazebo with hybrid_transition world
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'spawn_aeroterrabot.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_gazebo, 'worlds', 'hybrid_transition.world')}.items()
    )

    # Transform Controller node
    transform_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_transform, 'launch', 'transform_controller.launch.py')
        )
    )

    # Sequence commands to demonstrate the modes automatically
    
    # 1. Drive Forward (WHEEL_MODE)
    start_driving = TimerAction(
        period=5.0, # Wait for controllers to initialize
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/diff_drive_controller/cmd_vel_unstamped', 'geometry_msgs/msg/Twist', '"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'],
                output='screen'
            )
        ]
    )

    # 2. Switch to LEG_MODE
    to_leg_mode = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/diff_drive_controller/cmd_vel_unstamped', 'geometry_msgs/msg/Twist', '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'],
            ),
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/morph_mode', 'aeroterrabot_interfaces/msg/MorphMode', '{mode: 1}'], # 1 = LEG_MODE
                output='screen'
            )
        ]
    )

    # 3. Switch to FLIGHT_MODE
    to_flight_mode = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/morph_mode', 'aeroterrabot_interfaces/msg/MorphMode', '{mode: 3}'], # 3 = FLIGHT_MODE
                output='screen'
            )
        ]
    )

    # 4. Hover Offboard Node (simulate take-off)
    start_hover = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'aeroterrabot_flight_system', 'offboard_hover'],
                output='screen'
            )
        ]
    )

    # Note: Returning to leg and wheel modes from flight would require PX4 landing commands 
    # handled by the flight stack. The transition back can be triggered afterwards via CLI.

    return LaunchDescription([
        simulation,
        transform_controller,
        start_driving,
        to_leg_mode,
        to_flight_mode,
        start_hover
    ])
