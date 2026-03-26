import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Main Gazebo spawn launch for AeroTerraBot.
    
    Pipeline:
      1. Start Gazebo with world
      2. Spawn robot via robot_description
      3. Start robot_state_publisher
      4. Load and activate ros2_control controllers
    """

    pkg_description = get_package_share_directory('aeroterrabot_description')
    pkg_gazebo = get_package_share_directory('aeroterrabot_gazebo')
    pkg_control = get_package_share_directory('aeroterrabot_control')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_description, 'urdf', 'aeroterrabot.urdf.xacro')
    world_file = os.path.join(pkg_gazebo, 'worlds', 'aeroterrabot_world.world')
    controllers_yaml = os.path.join(pkg_control, 'config', 'controllers.yaml')
    rviz_config = os.path.join(pkg_description, 'rviz', 'aeroterrabot_visualization.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # --- Gazebo Server + Client ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true',
        }.items(),
    )

    # --- Robot State Publisher ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # --- Spawn Entity ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'aeroterrabot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
        ],
        output='screen',
    )

    # --- Controller Spawners ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_joint_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    transform_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['transform_joint_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    rotor_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rotor_velocity_controller', '--controller-manager', '/controller_manager', '--inactive'],
        output='screen',
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- Sequencing: spawn controllers after entity spawns ---
    spawn_then_broadcast = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    broadcast_then_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                diff_drive_spawner,
                leg_controller_spawner,
                transform_controller_spawner,
                rotor_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=world_file),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        gazebo,
        robot_state_publisher,
        spawn_entity,
        spawn_then_broadcast,
        broadcast_then_controllers,
        rviz_node,
    ])
