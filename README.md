# AeroTerraBot 🚁⚙️

AeroTerraBot is a complex, production-ready ROS 2 Humble workspace for a **Hybrid Morphing Autonomy Platform**. The unique robotics system is fully capable of dynamically transitioning between four distinct locomotion modes using `ros2_control`, a custom Transform Controller framework, and PX4 SITL physics simulation.

![AeroTerraBot Execution Screenshots](screenshots/execution.png)

## 🌟 Locomotion Modes
1. **WHEEL_MODE (0):** High-speed traversal using differential drive kinematics on 4 omni-directional wheels.
2. **LEG_MODE (1):** High-elevation posture maintaining stability over difficult uneven terrains utilizing 8 active joint position controllers.
3. **HYBRID_MODE (2):** A fused mixture of legs for clearance and wheels for combined traversal.
4. **FLIGHT_MODE (3):** Quadrotor arms dynamically deploy horizontally, instantly switching control logic over to PX4 rotor physics enabling full flight.

## 📁 Repository Structure Overview
* `aeroterrabot_description`: Dynamic parameterized XACRO geometry models, Gazebo hardware bridges, and URDF kinematics representing the custom hardware.
* `aeroterrabot_gazebo`: Bespoke worlds designed to challenge varying modes including rough off-road bumps, launch pads, and transition gates.
* `aeroterrabot_control`: Manages all active hardware-interfaces and seamlessly handles the hot-swapping sequence between drive interfaces to raw drone-rotors.
* `aeroterrabot_transform_controller`: Central C++ node overseeing safety conditions, firing leg retraction trajectory splines, folding quad arms, and maintaining mode state logic.
* `px4_msgs`: Built-from-source custom message bridge enabling PX4 Offboard communication natively through ROS 2.
* `aeroterrabot_bringup`: Heavy-hitter launch scripts mapping the whole infrastructure from visualization (RViz) to execution in one file.

## 🚀 Quickstart using Docker (Windows & Linux)
We employ a containerized approach scaling from underlying `osrf/ros:humble-desktop` libraries seamlessly directly to `host.docker.internal` X11 interfaces to reliably push physics displays on any Windows GUI.

1. **Spin up the Container**
```bash
docker-compose up -d --build
```
2. **Jump to the Workspace and Compile**
```bash
docker exec -it aeroterrabot_dev bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
```
3. **Launch the Demo** *(Requires XServer like VcXsrv on Windows)*
```bash
docker exec -it -e DISPLAY=host.docker.internal:0 aeroterrabot_dev bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch aeroterrabot_bringup transition_demo.launch.py"
```

## 📸 Screenshots
Execution screenshots are captured manually and submitted within the `screenshots/` directory showcasing the drone soaring off standard terrain!
