import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('aeroterrabot_gazebo')
    pkg_transform = get_package_share_directory('aeroterrabot_transform_controller')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Simulation environment & Robot
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'spawn_aeroterrabot.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Transform Controller node
    transform_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_transform, 'launch', 'transform_controller.launch.py')
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        simulation,
        transform_controller
    ])
