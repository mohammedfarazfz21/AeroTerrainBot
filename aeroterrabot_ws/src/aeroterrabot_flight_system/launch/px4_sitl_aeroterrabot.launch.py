import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('aeroterrabot_gazebo')
    
    # Start Gazebo with our robot via the gazebo package launch file
    gazebo_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'spawn_aeroterrabot.launch.py')
        )
    )

    # Start PX4 SITL
    # Assuming PX4-Autopilot is built in the environment
    px4_dir = os.environ.get('PX4_DIR', '/path/to/PX4-Autopilot')
    start_px4 = ExecuteProcess(
        cmd=[
            os.path.join(px4_dir, 'build/px4_sitl_default/bin/px4'),
            os.path.join(px4_dir, 'ROMFS/px4fmu_common'),
            '-s',
            os.path.join(px4_dir, 'ROMFS/px4fmu_common/init.d-posix/rcS')
        ],
        cwd=px4_dir,
        output='screen'
    )

    # Start MicroXRCEAgent (microRTPS bridge)
    start_micrortps_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # Node to map PX4 actuator outputs to ROS 2 controller commands
    # This acts as the bridge during FLIGHT_MODE
    actuator_bridge = Node(
        package='aeroterrabot_flight_system',
        executable='offboard_hover', # Simplified: hover node also handles bridging or we could use a dedicated bridge
        # In a real system, you'd have a dedicated bridge mapping actuator_controls_0 to joint velocities
        output='screen'
    )

    return LaunchDescription([
        gazebo_spawn,
        # start_px4, # Uncomment when PX4_DIR is correctly set
        # start_micrortps_agent, # Uncomment when MicroXRCEAgent is installed
        # actuator_bridge
    ])
