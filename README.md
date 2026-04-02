# AeroTerrainBot 🚁🚜

AeroTerrainBot is an innovative hybrid robotics platform capable of transforming between a quadrotor drone and an all-terrain wheeled vehicle, heavily inspired by the M4 Morphobot.

## 🌟 Capabilities

This robot utilizes a highly dynamic morphing structure:
- **Tank Mode (Ground)**: 4 omni-wheels drive the robot like a skid-steer vehicle.
- **Flight Mode (Air)**: Uses 2 morphing servos to roll the entire left and right booms upward by 90 degrees. The large wheels immediately transform into protective propeller guards, exposing the internal rotors for flight.
- **Stance Control**: Uses 2 linear actuators to extend the left and right booms further outward from the chassis, vastly widening its footprint for stable flight.

## 🛠️ Technology Stack

1. **ROS 2 Jazzy**: Core middleware.
2. **Docker**: Provided container setup (`osrf/ros:jazzy-desktop`).
3. **ros2_control**: Custom hardware interfaces with serial feedback/commands to the Teensy 4.0 main controller.

## 🚀 Setting Up the Simulation Environment

This workspace has been thoroughly configured for easy local development using Docker.

1. Install [VcXsrv (XLaunch)](https://sourceforge.net/projects/vcxsrv/) on Windows if you intend to visualize the robot.
2. Check the box **"Disable access control"** when setting up XLaunch.
3. Start the Docker environment:
```powershell
docker compose up -d
```

### Building the Workspace
```bash
docker exec -it aeroterrabot-aeroterrabot_dev-1 bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace/aeroterrabot_ws && colcon build --build-base build_ros --install-base install_ros"
```

### Running RViz (Visualization)
To test the morphology and view the URDF:
```bash
docker exec -it aeroterrabot-aeroterrabot_dev-1 bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/aeroterrabot_ws/install_ros/setup.bash && ros2 launch aeroterrabot_description display.launch.py"
```

## 📐 Hardware Architecture
* **Teensy 4.0**: Aggregates joint commands, serializes data, and feeds velocity/position commands to the 4 wheel motors, 4 rotor ESCs, 2 servos, and 2 linear actuators.
* **Arduino**: Serves as a sensor bridge gathering ultrasonic proximity and IMU data, streaming a synchronous serial byte frame (`OxBB 0x66`) straight into the `sensor_bridge_node` plugin in ROS 2.
