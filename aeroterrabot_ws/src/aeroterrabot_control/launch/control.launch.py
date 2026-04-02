import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("aeroterrabot_description"), "urdf", "robot.urdf.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_params_file = PathJoinSubstitution(
        [FindPackageShare("aeroterrabot_control"), "config", "controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    tank_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tank_drive_controller", "--controller-manager", "/controller_manager"],
    )

    morph_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["morph_controller", "--controller-manager", "/controller_manager"],
    )

    rotor_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rotor_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        tank_drive_controller_spawner,
        morph_controller_spawner,
        rotor_controller_spawner,
    ])
