import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Arguments ──────────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # ── Paths ──────────────────────────────────────────────────────────────────
    pkg_gazebo   = get_package_share_directory("aeroterrabot_gazebo")
    pkg_control  = get_package_share_directory("aeroterrabot_control")
    pkg_desc     = get_package_share_directory("aeroterrabot_description")

    world_file   = os.path.join(pkg_gazebo, "worlds", "aeroterrabot.world")
    controllers_yaml = os.path.join(pkg_control, "config", "controllers.yaml")
    urdf_file    = os.path.join(pkg_desc, "urdf", "robot.urdf.xacro")

    # ── Robot Description ──────────────────────────────────────────────────────
    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ", urdf_file,
            " use_sim_time:=true"
        ])
    }

    # ── Gazebo Sim ─────────────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    # ── Robot State Publisher ──────────────────────────────────────────────────
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # ── Spawn Robot into Gazebo ────────────────────────────────────────────────
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name",  "aeroterrabot",
            "-z",     "0.15",          # Spawn above ground (landing gear height)
        ],
        output="screen",
    )

    # ── Controller Spawners (delayed so controller_manager is ready) ───────────
    def make_spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
        )

    jsb_spawner   = make_spawner("joint_state_broadcaster")
    drive_spawner = make_spawner("tank_drive_controller")
    morph_spawner = make_spawner("morph_controller")
    rotor_spawner = make_spawner("rotor_controller")

    # Delay spawners to give controller_manager time to initialise
    delayed_spawners = TimerAction(
        period=4.0,
        actions=[jsb_spawner, drive_spawner, morph_spawner, rotor_spawner],
    )

    # ── Gazebo ↔ ROS Bridge ────────────────────────────────────────────────────
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        gz_sim,
        robot_state_pub,
        spawn_robot,
        gz_bridge,
        delayed_spawners,
    ])
