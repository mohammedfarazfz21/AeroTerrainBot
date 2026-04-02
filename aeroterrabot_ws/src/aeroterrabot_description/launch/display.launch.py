import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # ── PATHS ─────────────────────────────────────────────────────────────────
    pkg_description = get_package_share_directory('aeroterrabot_description')
    urdf_file       = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_description, 'rviz', 'view_robot.rviz')

    # ── ARGUMENTS ─────────────────────────────────────────────────────────────
    # None for this simple display launch

    # ── NODES ─────────────────────────────────────────────────────────────────

    # 1. Robot State Publisher
    robot_description_config = Command(['xacro ', urdf_file])
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # 2. Joint State Publisher GUI (For testing kinematics without controllers)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # 3. RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # ── LAUNCH DESCRIPTION ────────────────────────────────────────────────────
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
