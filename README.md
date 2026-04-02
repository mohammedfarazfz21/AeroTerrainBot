# AeroTerrainBot 🚁🚜

AeroTerrainBot is an innovative hybrid robotics platform capable of transforming between a quadrotor drone and an all-terrain wheeled vehicle, heavily inspired by the M4 Morphobot.

## 🌟 Visualisations

### Ground Mode vs Flight Mode (RViz)
| Tank Mode (Ground) | Flight Mode (Air) |
| :---: | :---: |
| ![Tank Mode](docs/images/tank_mode_rviz.png) | ![Flight Mode](docs/images/flight_mode_rviz.png) |
| *Wheels down, booms horizontal for skid-steer traversal* | *Booms rotated 90° up, wheels acting as prop-guards* |

### Gazebo Harmonic Simulation
![Gazebo Simulation](docs/images/gazebo_simulation.png)
*AeroTerraBot spawning and driving in Gazebo Harmonic with active physics and sensors.*

## ✨ New Features (Latest Updates)

- **Gazebo Harmonic Integration**: Full migration to modern Gazebo Sim (Harmonic) with native `gz_ros2_control` support.
- **Hybrid Controller Switching**: Seamlessly switch between `tank_drive_controller` (skid-steer) and `morph_controller` (trajectory-based transformation).
- **Integrated Sensor Suite**: Native support for IMU, Lidar (gpu_lidar), and Camera sensors with ROS-GZ bridging.
- **Hardware Abstraction**: Parameterized XACRO structure that toggles between `GazeboSimSystem` for simulation and `AeroTerraBotSystemInterface` for real-world Teensy 4.0 control.
- **Visual Feedback**: Added visual cues (wheel spokes) and optimized friction parameters for realistic physics interactions.

## 🛠️ Technology Stack

1. **ROS 2 Jazzy**: Core middleware.
2. **Gazebo Harmonic**: Physics engine.
3. **ros2_control**: Modular hardware abstraction layer.
4. **Docker**: Unified development environment.

## 🚀 Setting Up the Simulation Environment

### 1. Build the Workspace
Execute the build inside the dev container:
```powershell
docker exec -it aeroterrabot-aeroterrabot_dev-1 bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace/aeroterrabot_ws && colcon build --build-base build_ros --install-base install_ros"
```

### 2. Launch Simulation
Start the Gazebo world with the robot and bridge:
```powershell
docker exec -it aeroterrabot-aeroterrabot_dev-1 bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/aeroterrabot_ws/install_ros/setup.bash && ros2 launch aeroterrabot_gazebo gazebo.launch.py"
```

### 3. Drive the Robot
In a separate terminal, use the keyboard to drive:
```powershell
docker exec -it aeroterrabot-aeroterrabot_dev-1 bash -c "source /opt/ros/jazzy/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/tank_drive_controller/cmd_vel"
```

## 📐 Hardware Architecture
* **Teensy 4.0**: Aggregates joint commands and manages motor/servo signals.
* **Controller Interface**: Communicates via serial using the `aeroterrabot_hardware` package.
