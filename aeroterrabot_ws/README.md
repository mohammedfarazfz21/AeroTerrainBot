# AeroTerraBot Workspace

This repository contains the complete ROS 2 Humble workspace for **AeroTerraBot**, a hybrid morphing autonomy platform capable of wheel mobility, leg elevation stabilization, hybrid terrain traversal, and quadrotor flight.

## Features
- **Wheel Mode:** Fast traversal using differential drive kinematics.
- **Leg Mode:** Elevation and stability on uneven terrain using joint position controllers.
- **Hybrid Mode:** Combined wheel and leg usage.
- **Flight Mode:** Quadrotor propulsion integration with PX4 SITL.
- **Transform Controller:** Safely orchestrates morphing sequences and ros2_control controller switching.
- **Navigation:** Integrated Nav2 stack setup.
- **Simulation:** Pre-configured Gazebo environments (flat terrain, off-road bumps, landing pads).
- **Sensors:** 360-degree LiDAR, RGB Camera, and IMU.

## System Requirements
- **OS:** Ubuntu 22.04 Setup / Windows WSL2 or Native Windows ROS 2 Humble
- **ROS 2:** Humble Hawksbill
- **Simulation:** Gazebo Fortress / Gazebo Harmonic
- **Flight Stack:** PX4 Autopilot SITL
- **Bridge:** MicroXRCEAgent

## Dependencies
Install common ros2 packages:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher \
  ros-humble-diff-drive-controller ros-humble-joint-trajectory-controller \
  ros-humble-velocity-controllers ros-humble-position-controllers \
  ros-humble-nav2-bringup ros-humble-slam-toolbox
```

Clone PX4 SITL and build specifically for gazebo:
```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl gazebo
```

## Build Instructions
Navigate to the root of your workspace:
```bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Commands

### 1. Basic Simulation Bringup
Launches Gazebo, spawns the robot, loads hardware interfaces, and brings up the transformation manager node.
```bash
ros2 launch aeroterrabot_bringup bringup.launch.py
```

### 2. Navigation Bringup
Launches the basic simulation + the Nav2 Stack.
```bash
ros2 launch aeroterrabot_bringup navigation.launch.py
```

### 3. Flight SITL Integration
Launches Gazebo alongside PX4 SITL and offboard hover controls.
*Note: Make sure PX4_DIR env variable is set to your PX4-Autopilot folder.*
```bash
ros2 launch aeroterrabot_flight_system px4_sitl_aeroterrabot.launch.py
```

### 4. Full Transition Demo Sequence
Automatically drives, stops, stands on legs, transforms to flight mode, and executes hover offboard node.
```bash
ros2 launch aeroterrabot_bringup transition_demo.launch.py
```

## Mode Switching Commands
You can manually command the transformation controller via the `/morph_mode` topic.

**0 = WHEEL_MODE, 1 = LEG_MODE, 2 = HYBRID_MODE, 3 = FLIGHT_MODE**

```bash
# Switch to Wheel Mode
ros2 topic pub --once /morph_mode aeroterrabot_interfaces/msg/MorphMode "{mode: 0}"

# Switch to Leg Mode
ros2 topic pub --once /morph_mode aeroterrabot_interfaces/msg/MorphMode "{mode: 1}"

# Switch to Hybrid Mode
ros2 topic pub --once /morph_mode aeroterrabot_interfaces/msg/MorphMode "{mode: 2}"

# Switch to Flight Mode (Arms Deploy)
ros2 topic pub --once /morph_mode aeroterrabot_interfaces/msg/MorphMode "{mode: 3}"
```

## Teleoperation
You can drive the robot in wheel_mode using the standard `teleop_twist_keyboard` mapped to diff_drive controller:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

## Subsystem Architecture
- `aeroterrabot_description`: XACRO definition of the hybrid robot with sensors.
- `aeroterrabot_gazebo`: Launch setups for multiple Gazebo worlds.
- `aeroterrabot_control`: Detailed `controllers.yaml` for ros2_control configuration.
- `aeroterrabot_transform_controller`: Central mode-manager logic handling arm deployment and leg retractions via Action/Trajectory interfaces.
- `aeroterrabot_flight_system`: PX4 integration mappings and hover scripts.
- `aeroterrabot_navigation`: Nav2 `nav2_params.yaml` implementation.
- `aeroterrabot_interfaces`: Custom mode and status `.msg` definitions.
- `aeroterrabot_bringup`: Common integration pipelines.
