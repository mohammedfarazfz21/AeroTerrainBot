FROM osrf/ros:jazzy-desktop

# ── Install dependencies ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    git \
    && rm -rf /var/lib/apt/lists/*

# ── Workspace setup ───────────────────────────────────────────────────────────
WORKDIR /workspace
ENV SHELL /bin/bash

# Source ROS 2 setup automatically
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
