# ROS 2 Humble, Gazebo Classic 11 and every package the stack loads (the apt list of docs/02-prerequisites.md, task 2.1).
#   docker build -t go2-nav-sim --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) .devcontainer
FROM osrf/ros:humble-desktop-full-jammy
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-robot-localization \
      ros-humble-pointcloud-to-laserscan ros-humble-tf2-sensor-msgs ros-humble-xacro \
      ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers \
      ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2-storage-mcap ros-humble-rosbag2-compression-zstd \
      python3-colcon-common-extensions python3-numpy python3-scipy python3-opencv python3-pil ffmpeg git \
    && rm -rf /var/lib/apt/lists/*
# A user with your UID, so that files the build writes into the mounted repository belong to you; video for /dev/dri.
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID dev && useradd --uid $USER_UID --gid $USER_GID --groups video -m -s /bin/bash dev
USER dev
WORKDIR /home/dev/go2-nav-sim
