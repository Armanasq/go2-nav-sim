# 0. What you will build

A simulated Unitree Go2 that walks across a room to a goal, using a 3D lidar, an IMU and Nav2. The pose estimate that Nav2
uses is computed from the robot's own sensors. The simulator's ground truth is used once, to initialise AMCL, and after that
only recorded to score the run, so every run ends with numbers as well as a video.

![The Unitree Go2 rendered in MuJoCo, three-quarter and side view](img/c00-00-go2-render.png)

*The Go2 as modelled in MuJoCo Menagerie (`modules/assets/unitree_go2`, used by the modules); the ROS stack simulates the URDF from `unitree-go2-ros2` in Gazebo.*

<video controls muted playsinline src="../media/c00-00-a2b-start.mp4"></video>

*From 0:03 to 0:15 of the dashboard video `media/go2_a2b_dashboard.mp4`: the robot stands at A, Nav2 publishes a plan (amber) and the robot starts to walk.*

![Top view of the arena with obstacles, start A, goal B and the ground-truth path](img/c00-00-arena.png)

*The room, from `site/data/runs/w2/run.json`: start A at (-4.5, -3.5), goal B at (4.0, 3.0), six obstacles, and the path the robot walked in run w2.*

## The stack in one paragraph

Gazebo Classic simulates the robot, its CHAMP gait controller and the sensors. A lidar odometry node registers each lidar scan
against a local map, with yaw taken from the gyro. An EKF fuses that with the IMU and publishes the `odom` frame. AMCL localises
on a known map and Nav2 plans and drives. A second mapper, `slam_toolbox`, builds its own map in the background so you can
compare it with the real layout. Every run is recorded to a rosbag, scored against ground truth and rendered to a video.

![ROS 2 graph of the stack: Gazebo, CHAMP, lidar odometry, EKF, AMCL, slam_toolbox, Nav2 and the topics between them](img/c00-00-ros-graph.png)

*The nodes and topics of the running stack, read from `src/go2_nav_sim/launch/*.py`; the fill says where each node comes from. `scripts/record_bag.sh` records the sensor, odometry, map, plan and command topics.*

## Route

The tutorial has two parts. The first gets a legged robot into ROS 2 and walking; the second gives it tasks.

| Part | Chapter | You end with |
|---|---|---|
| Foundations | [1. Legged robot foundations](01-legged-robot-foundations.md) | the layers between a URDF and a walking robot |
| | [2. Prerequisites](02-prerequisites.md) | Ubuntu 22.04, ROS 2 Humble and the packages, checked |
| | [3. Install and first run](03-install-and-first-run.md) | the full stack running and reaching the goal |
| | [4. ROS 2 from scratch](04-ros2-from-scratch.md) | the running graph inspected, QoS and sim time understood, a node of your own built and tested |
| | [5. Frames and TF2](05-frames-and-tf2.md) | the robot's TF tree read, a transform derived and checked by hand, one wrong transform found |
| | [6. Import the Go2](06-import-the-go2.md) | how the robot description, ros2_control and CHAMP fit together |
| | [7. Make it walk](07-make-it-walk.md) | the robot walking on your commands, gait parameters understood |
| Tasks | [8. Robot and sensors](08-robot-and-sensors.md) | lidar, RGB-D and IMU publishing in the right frames |
| | [9. State estimation](09-state-estimation.md) | odometry from lidar and IMU, scored against ground truth |
| | [10. Mapping and navigation](10-mapping-and-navigation.md) | Nav2 driving the robot from A to B, a map built alongside |
| | [11. Record and visualise](11-record-and-visualise.md) | a rosbag, its metrics and a dashboard video |
| Help | [12. Troubleshooting](12-troubleshooting.md) | every failure met while building this, with the fix |

Hardware: about 5 GB of disk plus about 0.5 GB per minute of recording (the bag grows at about 8 MB/s), 8 or more CPU threads,
and a GPU for the cameras and the GPU lidar.

## What is real and what is not

| | |
|---|---|
| Simulated | the robot, its gait, the sensors and the room. Nothing here drives a physical Go2. |
| Used unmodified | Nav2, AMCL, `robot_localization`, `slam_toolbox`, `pointcloud_to_laserscan` |
| Written for this project | the lidar odometry (`lidar_odom.py` and `registration.py`), the launch and bring-up scripts, the evaluation and the dashboard. The registration has unit tests that run without ROS. |
| Shortcuts | AMCL is started from the true pose, and navigation uses a map that is known in advance. [Chapter 10](10-mapping-and-navigation.md) explains both. |

## How to read this tutorial

Each chapter ends in a state you can check with a command. If a check fails, [chapter 12](12-troubleshooting.md) lists every
failure met while building this, with its cause and fix.

Next: [1. Legged robot foundations](01-legged-robot-foundations.md)
