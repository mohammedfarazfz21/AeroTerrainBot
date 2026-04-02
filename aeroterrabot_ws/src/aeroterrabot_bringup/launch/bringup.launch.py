import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # ── PATHS ─────────────────────────────────────────────────────────────────
    pkg_description = get_package_share_directory('aeroterrabot_description')
    pkg_control     = get_package_share_directory('aeroterrabot_control')
    pkg_hardware    = get_package_share_directory('aeroterrabot_hardware')
    
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')

    # ── ARGUMENTS ─────────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ── NODES ─────────────────────────────────────────────────────────────────

    # 1. Robot State Publisher (Calculates TFs from Joint States)
    robot_description_config = Command(['xacro ', urdf_file])
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
    )

    # 2. Controller Manager (Loads and runs ros2_control controllers)
    # Note: In hardware, robot_state_publisher must be running first.
    # The controller manager is often launched as part of the hardware system.
    controller_params_file = os.path.join(pkg_control, 'config', 'controllers.yaml')

    node_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_config},
            controller_params_file
        ],
        output='screen'
    )

    # 3. Spawners for the controllers
    # These effectively "activate" the controllers defined in controllers.yaml
    
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    spawn_tank_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tank_drive_controller'],
        output='screen',
    )

    spawn_morph_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['morph_controller'],
        output='screen',
    )

    # 4. Sensor Bridge Node (Arduino communication)
    node_sensor_bridge = Node(
        package='aeroterrabot_hardware',
        executable='sensor_bridge_node',
        name='sensor_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 115200
        }]
    )

    # ── LAUNCH DESCRIPTION ────────────────────────────────────────────────────
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        node_robot_state_publisher,
        node_controller_manager,
        spawn_joint_state_broadcaster,
        spawn_tank_drive_controller,
        spawn_morph_controller,
        node_sensor_bridge
    ])
