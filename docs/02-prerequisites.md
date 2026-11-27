# 2. Prerequisites

Tested on Ubuntu 22.04 with ROS 2 Humble and Gazebo Classic 11, on a laptop with an RTX 4090 and 32 threads. The stack ran at 1.0x
real time there. A discrete GPU is not required, but three cameras (RGB-D, chase and spectator) and a 16-beam GPU lidar are slow on integrated
graphics.

## Simulator version

The stack runs on Gazebo Classic 11, which reached end of life on 31 January 2025 and is not available for ROS 2 Jazzy. It still
installs and works on Humble, which is why this tutorial uses it, but it receives no fixes. The current pairing is ROS 2 Jazzy
with Gazebo Harmonic; porting to it is on the roadmap (module 03). The algorithms, configurations
and evaluation in this repository do not depend on the simulator.

![Software layers: Ubuntu 22.04, ROS 2 Humble with Cyclone DDS, Gazebo Classic 11, the apt packages by role, and this repository on top](img/c00-02-software-stack.png)

*What this chapter installs, bottom to top, with the apt packages grouped by the part of the stack that uses them. Dashed: the current pairing, not used here.*

## System packages

```bash
sudo apt install \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-pointcloud-to-laserscan ros-humble-tf2-sensor-msgs ros-humble-xacro \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2-storage-mcap ros-humble-rosbag2-compression-zstd \
  python3-colcon-common-extensions python3-numpy python3-scipy python3-opencv python3-pil ffmpeg git
```

Without sudo, skip `ros-humble-gazebo-ros-pkgs`; `scripts/setup.sh` downloads the three packages that carry the Gazebo sensor plugins
with `apt-get download` and unpacks them into `overlay/`. Everything else in the list must already be installed.

The cameras and the GPU lidar need an OpenGL context, so the machine needs a display (a desktop session, or a virtual one such
as Xvfb; I only tested a desktop session). Gazebo itself runs without a window.

## Check your machine

```bash
ls /opt/ros/humble/lib/libgazebo_ros_camera.so   # sensor plugins (or use the overlay)
ros2 pkg prefix nav2_bringup slam_toolbox robot_localization pointcloud_to_laserscan
ls /opt/ros/humble/lib/librmw_cyclonedds_cpp.so
python3 -c "import rosbag2_py, scipy, cv2, PIL; print('python ok')"
```

## Two environment traps

- A Python virtualenv on `PATH` breaks the build. `rosidl` imports `em` (empy) from the system Python. `env.sh` removes any
  `.venv` entry from `PATH`; if you build by hand, deactivate first.
- Fast DDS drops service replies on a busy machine. Nav2 starts its nodes with lifecycle service calls, and a lost reply aborts
  the bring-up. `env.sh` selects Cyclone DDS on the loopback interface. Do not skip this step.

![Two panels: a virtualenv python shadowing the system python during the build, and a lifecycle service reply lost under Fast DDS](img/c00-02-env-traps.png)

*How each trap fails and the `env.sh` line that prevents it. The error messages are the ones listed in [chapter 12](12-troubleshooting.md).*

Next: [3. Install and first run](03-install-and-first-run.md)
