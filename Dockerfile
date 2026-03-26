FROM osrf/ros:humble-desktop

# Avoid user interaction during apt installations
ENV DEBIAN_FRONTEND=noninteractive

# Update and install required dependencies for AeroTerraBot
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-diff-drive-controller \
    ros-humble-joint-trajectory-controller \
    ros-humble-velocity-controllers \
    ros-humble-position-controllers \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-xacro \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Setup bashrc to always source ROS 2 and the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "if [ -f /aeroterrabot_ws/install/setup.bash ]; then source /aeroterrabot_ws/install/setup.bash; fi" >> ~/.bashrc

# Set the working directory
WORKDIR /aeroterrabot_ws

# Optional: You can copy your local workspace to the image directly
# COPY ./aeroterrabot_ws /aeroterrabot_ws

# Default command (gives you an interactive terminal)
CMD ["bash"]
