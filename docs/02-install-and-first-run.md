# 2. Install and first run

```bash
git clone https://github.com/Armanasq/go2-nav-sim.git && cd go2-nav-sim
scripts/setup.sh          # fetch upstream at a pinned commit, apply the patch, build (about 5 minutes)
source env.sh
scripts/up.sh             # start everything, with readiness checks
```

`setup.sh` does three things:

1. Fetches [anujjain-dev/unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2) (Go2 description and CHAMP gait
   controller for Humble) at commit `fdac259` into `src/third_party/` and applies `patches/upstream.patch`. The patch is small,
   see [what it changes](#what-the-patch-changes).
2. Installs the Gazebo sensor plugins into `overlay/` if the system lacks them.
3. Runs `colcon build --symlink-install`.

`up.sh` starts three launch files in order and waits for each to be ready:

| Step | Launch file | Provides |
|---|---|---|
| 1 | `sim.launch.py` | Gazebo arena, Go2 with CHAMP, lidar, RGB-D camera, IMU, ground-truth pose |
| 2 | `estimation.launch.py` | `/scan`, lidar odometry, EKF, the `odom` to `base_footprint` transform |
| 3 | `nav.launch.py` | Nav2, AMCL on the arena map, a passive `slam_toolbox` |

When it prints `stack up`, check it:

```bash
source env.sh
ros2 topic hz /lidar/points /camera/image_raw /imu/data    # 10, 10 and 100 Hz
ros2 run tf2_ros tf2_echo map base_footprint                # a pose, updating
```

Gazebo runs without a window. To see it, launch `sim.launch.py headless:=False` yourself.

## Send the robot to the goal

```bash
python3 src/go2_nav_sim/scripts/go_to_goal.py
# RESULT SUCCEEDED  sim_time 57.5 s  ground-truth final error 0.27 m
```

The robot starts on the green disc A and walks to the red disc B at (4.0, 3.0). Pass `x y yaw` for another goal.
Stop everything with `scripts/stop.sh`.

## What the patch changes

Upstream targets a teleoperation demo. Three things had to change for this stack:

- `champ_gazebo/launch/gazebo.launch.py` loaded controllers with `ros2 control load_controller`. That command comes from
  `ros2controlcli`, which is often not installed, and without it the legs never receive commands. The patch uses the
  `controller_manager` spawner instead.
- The same file started a `contact_sensor` node that used over 200% of a core for a feature that is switched off. Removed.
- `champ_bringup/launch/bringup.launch.py` gets a `gt_odom` argument that disables CHAMP's leg-odometry EKF, so another source can
  own the `odom` transform.

Next: [3. Robot and sensors](03-robot-and-sensors.md)
