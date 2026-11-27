# The real Unitree Go2 after simulation: sources and what to take from them

Researched 2026-11-28. Angle: what a learner meets when moving from simulation to the real Go2, including the official
SDKs, ROS 2 interface, sensors, network, safety, and the simulator and ROS 2 versions the community uses.

Tags: **V** means I opened the primary source and read the relevant part. **S** means search snippet only. **U** means
unverified. **(my inference)** means my own reasoning. I took star counts and last-push dates from the GitHub API on
2026-11-28.

No book or structured course on the real Go2 with ROS 2 turned up in this pass, so the list has none. The material below
is all repositories, vendor pages and lab wikis.

---

## 1. Annotated sources

### 1.1 Unitree Document Center (support.unitree.com/home/en/developer): **U, could not open**
https://support.unitree.com/home/en/developer/Quick_start (also `sports_services`, `Basic_services`, `ROS2_service`,
`Get_remote_control_status`, `ObstaclesAvoidClient`, `VuiClient`)

This is the official developer documentation that every Unitree repository points to. It is a JavaScript single-page app.
WebFetch returns only the title "宇树科技 文档中心". A headless browser is blocked with a Tencent EdgeOne "Access
Restricted" page, so automated clients cannot read it. Everything in this note that the repos attribute to these pages
(Move ranges, frame definitions, safety prerequisites) is tagged U or S until someone reads it in a normal browser. A
search snippet from a Chinese summary of the "sport service V2.0" interface (**S**) gives approximate Move limits of vx
about -2.5 to 3.8 m/s, vy -1.0 to 1.0 m/s and vyaw -4 to 4 rad/s. The same snippet says a command is unfiltered and
stays in effect for about 1 s, and that the interface needs Go2 EDU software 1.1.6 or later. Check this page by hand
before a course quotes any of these numbers.

### 1.2 unitreerobotics/unitree_ros2: **V**
https://github.com/unitreerobotics/unitree_ros2 (905 stars, last push 2026-07-02)

This is Unitree's ROS 2 interface. It is not a driver. Because the robot's own middleware is CycloneDDS, ROS 2 nodes
using `rmw_cyclonedds_cpp` can read and write the robot's DDS topics directly with Unitree's message packages
(`unitree_go`, `unitree_api`, `unitree_hg`).

**Setup:**
- Tested on Foxy (Ubuntu 20.04) and Humble (Ubuntu 22.04, marked "recommend").
- The robot runs CycloneDDS 0.10.2. On Foxy you build Cyclone 0.10.x from source in a terminal where ROS is not
  sourced. On Humble you skip that step.
- Give the PC a static IP of 192.168.123.99/24 on the wired interface.
- Source `setup.sh`, which sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and a `CYCLONEDDS_URI` naming the network
  interface. `setup_local.sh` uses `lo` for simulation.
- The shipped scripts hard-code `/opt/ros/foxy`. Open PR #13 changes this to `$ROS_DISTRO`.

**State topics:**
- `sportmodestate` and `lf/sportmodestate`, where "lf" means low frequency. Fields: mode, gait_type, position[3],
  velocity[3], yaw_speed, body_height, foot_force[4], foot_position_body[12], foot_speed_body[12], range_obstacle[4],
  and an `IMUState` (quaternion, gyroscope, accelerometer, rpy, temperature).
- `lowstate` and `lf/lowstate`: 20 motor states (q, dq, ddq, tau_est, temperature), BMS, foot_force and foot_force_est,
  power_v and power_a, and wireless_remote.
- `/wirelesscontroller`: joystick axes in [-1, 1] and key bits.
- `utlidar/cloud`: the L1 point cloud, frame_id `utlidar_lidar`.

**Control:**
- High-level: publish `unitree_api/msg/Request` on `/api/sport/request`. The reply comes on `/api/sport/response`. A
  helper `SportClient` builds requests by API ID, for example Damp 1001, BalanceStand 1002, StopMove 1003, StandUp
  1004, StandDown 1005, RecoveryStand 1006, Move 1008, SpeedLevel 1015, ClassicWalk 2049 and SwitchAvoidMode 2058.
- Low-level: publish `unitree_go/msg/LowCmd` on `/lowcmd`. Each motor takes q, dq, tau, kp and kd, and the message
  needs a CRC (`motor_crc.cpp`).

**Gaps:**
- No `geometry_msgs/Twist` `/cmd_vel` bridge is merged. Open PRs #17 and #21 add one.
- No Jazzy support is merged. Open PR #29, from 2026-11-16, adds native Jazzy builds, a `docs/jazzy.md` file and
  best-effort (sensor-data) QoS for telemetry subscribers.
- No tf or URDF is published.

### 1.3 unitreerobotics/unitree_sdk2: **V**
https://github.com/unitreerobotics/unitree_sdk2 (1,462 stars, last push 2026-11-19, release 2.0.2 on 2025-07-07)

This is the C++ SDK behind everything else. It is built with CMake on Ubuntu 20.04 with gcc 9.4, for aarch64 and
x86_64, and installs to `/opt/unitree_robotics` by default.

The Go2 headers include `sport`, `obstacles_avoid`, `robot_state`, `video`, `vui`, `utrack` and `config` clients. The
Go2 examples are `go2_sport_client`, `go2_low_level`, `go2_stand_example`, `go2_trajectory_follow`, `go2_video_client`
and `go2_vui_client`. `SportClient` exposes `Move(float vx, float vy, float vyaw)`, `StopMove()`, `Damp()`,
`BalanceStand()`, `SpeedLevel(int)`, `FreeWalk()` and `ClassicWalk(bool)`.

Two details matter for safety:
1. `go2_stand_example` calls `MotionSwitcherClient::ReleaseMode()` to switch off the built-in motion service before it
   sends joint commands. It also prints "WARNING: Make sure the robot is hung up or lying on the ground. Press Enter to
   continue...".
2. The low-level example initialises every motor with `q = PosStopF (2.146E+9)` and `kp = 0`, which holds the joint
   passive. It ramps up to a low gain of `Kp = 5`.

DDS topic names inside the SDK carry an `rt/` prefix, for example `rt/lowcmd` and `rt/lowstate`. The ROS 2 names in
1.2 have no prefix because ROS 2 adds `rt/` to topic names on the DDS wire (my inference, consistent with both repos).

### 1.4 unitreerobotics/unitree_sdk2_python: **V**
https://github.com/unitreerobotics/unitree_sdk2_python (852 stars, last push 2026-09-21)

This is the Python binding of sdk2. It needs `cyclonedds==0.10.2`. A common install error is "Could not locate
cyclonedds", which the README fixes by building Cyclone 0.10.x and setting `CYCLONEDDS_HOME`.

Every example takes the network interface as its argument, for example `sportmode_test.py enp2s0`. The examples cover
stand up and down, velocity move, attitude, trajectory follow, front camera via OpenCV, the obstacle-avoidance switch and
the light and volume client.

The low-level example says to "use the app to turn off the high-level motion service (sport_mode) to prevent
conflicting instructions", and to keep test gains low "for safety, set kp=10, kd=1". This is the shortest path from zero
to a moving real Go2. It is Python and has no ROS.

### 1.5 unitreerobotics/unitree_mujoco: **V**
https://github.com/unitreerobotics/unitree_mujoco (1,316 stars, last push 2026-09-07)

This is Unitree's own simulator. It is MuJoCo plus an sdk2 bridge, in C++ (recommended) or Python. It speaks the same
DDS messages as the robot, so the same binary runs in simulation and on hardware.

It supports only low-level development: `LowCmd`, `LowState` and `SportModeState`. There is no sport-mode or
high-level walking service. The README notes that on the real robot `SportModeState` cannot be read once the built-in
motion service is off, while the simulator keeps publishing it.

For sim-to-real, use DDS domain 1 on `lo` for simulation and domain 0 on the real interface for the robot. The C++
example switches between the two with `ChannelFactory::Instance()->Init(1, "lo")` and `Init(0, "enp3s0")`. Go2 uses the
`unitree_go` IDL. A gamepad can stand in for the Unitree wireless controller, and a terrain generator is included.

### 1.6 abizovnuralem/go2_ros2_sdk: **V**
https://github.com/abizovnuralem/go2_ros2_sdk (1,085 stars, last push 2026-07-13)

This is the most-starred community ROS 2 driver. It works with AIR, PRO and EDU over WebRTC (Wi-Fi, the protocol the
phone app uses) or CycloneDDS (Ethernet). It is tested on Humble, Iron and Rolling, and an issue moved CI from Iron to
Jazzy (#126).

`robot.launch.py` starts robot_state_publisher with a URDF, the go2_driver_node, a lidar-to-PointCloud2 node,
pointcloud_to_laserscan, the front camera, joy with teleop_twist_joy, twist_mux, a foxglove bridge, slam_toolbox
(online async) and Nav2. That makes it a complete map-then-navigate reference on the real robot.

Real-world limits the README states:
- On firmware v1.1.7 over WebRTC, the lidar arrives at about 7 Hz and joint states at 1 Hz.
- Close the phone app before connecting.
- Firmware 1.1.15 and later need a per-device AES-128 key for the WebRTC handshake.

Its operating advice is concrete. Mark a "dock" on the floor with tape. Check that the robot is oriented correctly
against a reloaded map. "Follow your dog and pick it up when it is about to do something silly." Most navigation faults
come from a wrong map, a wrong initial pose, or a failed plan.

### 1.7 legion1581/unitree_webrtc_connect: **V (README only)**
https://github.com/legion1581/unitree_webrtc_connect (428 stars, last push 2026-11-25)

This is a Python WebRTC driver for the Go2 AIR, PRO and EDU, using the same protocol as the phone app. Its table maps
firmware to authentication: 1.0.19 to 1.1.14 use a static key, and 1.1.15 and later use a per-device AES-128 key. On the
Go2 it supports the data channel, sport control, video, lidar decoding, VUI and the obstacle-avoidance API.

It matters because AIR and PRO units have no Ethernet developer access in the vendor's model, so Wi-Fi WebRTC is how
learners with non-EDU robots get ROS 2 data at all (my inference from 1.6 and 1.7).

### 1.8 jizhang-cmu/autonomy_stack_go2: **V (README and two source files)**
https://github.com/jizhang-cmu/autonomy_stack_go2 (578 stars, last push 2026-02-18)

This is a full autonomy stack from CMU: Point-LIO SLAM on the built-in L1, terrain analysis, collision avoidance,
waypoint following and a route planner. It runs in a Unity simulation (Foxy or Humble) and on the real Go2, either on
the onboard computer (Ubuntu 20.04 and Foxy, preinstalled) or on an external PC.

**Real-robot setup:**
- External PC on the rear-facing Ethernet port at IP 192.168.123.100/24 with gateway 192.168.123.1.
- Source Unitree's DDS setup script in every terminal.
- Calibrate the IMU inside the L1 once per robot: the robot steps for 2 s, stands for 10 s, then spins for 20 s, and
  the result is saved to `imu_calib_data.yaml`.
- Record bags with `ros2 bag record /utlidar/cloud /utlidar/imu`.
- When replaying bags, disconnect from the robot, "or the onboard sensor data may come in".
- An H.264 republisher turns the camera stream into `/camera/image/raw`.
- It offers three modes: smart joystick (teleop with collision avoidance), waypoint, and manual (no avoidance).

**From the source (`transform_everything.py`):**
- The L1 cloud and IMU are rotated into the body frame by a pitch of 2.8782 rad, about 164.9°, with an extra yaw of π
  for the IMU. My reading is that the lidar is mounted inverted and tilted.
- Timestamps are shifted by a one-off offset between the host clock and the robot's stamps.

### 1.9 Unitree Go2 product page and L1 LiDAR page: **V**
https://www.unitree.com/go2 and https://www.unitree.com/LiDAR

**Go2 page:**
- Variants AIR, PRO, X and EDU. Weight about 15 kg. Standing size 70 × 31 × 40 cm.
- 12 joint motors with about 45 N·m peak torque.
- HD wide-angle camera on all variants. Foot-end force sensors on EDU only.
- 8000 mAh battery (1 to 2 h) on AIR, PRO and X, and 15000 mAh (2 to 4 h) on EDU.
- Wi-Fi 6, Bluetooth 5.2, and 4G with GPS on PRO, X and EDU.
- An Orin module (40 to 100 TOPS) is an optional accessory.
- The page now lists the **4D LiDAR L2** (360° × 96°, 0.05 m minimum range), not the L1. Units in the field may
  therefore carry either lidar (my inference).

**L1 page:**
- 360° × 90° field of view, 0.05 m blind zone.
- 30 m range at 90% reflectivity (RM model).
- 21,600 effective points/s, 11 Hz circumferential scan.
- Built-in 3-axis accelerometer and gyroscope.

### 1.10 unitreerobotics/unilidar_sdk and point_lio_unilidar: **V**
https://github.com/unitreerobotics/unilidar_sdk (147 stars, last push 2024-02-18) and
https://github.com/unitreerobotics/point_lio_unilidar (540 stars, last push 2025-06-05)

The SDK is for the standalone L1 and offers C++, ROS 1 and ROS 2 interfaces. It defines the lidar frame:
- Origin at the centre of the bottom mounting face.
- +X opposite the cable, +Z up, right-handed.
- IMU axes parallel to the lidar axes, with the IMU at [-0.007698, -0.014655, 0.00667] m.

`point_lio_unilidar` adapts Point-LIO to the L1 and L2. It is ROS 1 Noetic only. On the Go2 the L1 is internal and
comes out on DDS as `utlidar/cloud` and `utlidar/imu`, so the SDK is not needed there. The frame definition and the IMU
extrinsic still apply (CMU's config reuses the same extrinsic_T with the sign flipped).

### 1.11 Southampton Robot Guides (SooratiLab, 2025): Go2 ROS control and troubleshooting: **V**
https://robots.ecs.soton.ac.uk/go2/ros_control and https://robots.ecs.soton.ac.uk/go2/troubleshooting

This is a university lab's operating guide for a Go2 EDU running a ROS 2 driver on the Jetson at 192.168.123.18. The
laptop uses static IP 192.168.123.222, because the Go2 has no DHCP. Settings are `ROS_DOMAIN_ID=0`,
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and CycloneDDS 0.10.2, which must match the Jetson.

**Interface:** `/cmd_vel` (Twist), `/odom`, `/scan` (use reliable QoS if nothing arrives), `/joint_states`, and a
`/mode` service with modes such as `damp`, `balance_stand`, `stop_move` and `sit`.

**Safety rules:**
- `/cmd_vel` has no collision avoidance, so clear the area and keep people away.
- `damp` is the emergency stop.
- Never run `wallow` or `front_flip`.
- Run `stand_down` before power-off.

**Troubleshooting:**
- Check `echo $CYCLONEDDS_URI` and that the interface named in it is correct.
- Run `ros2 daemon stop` then `ros2 daemon start`.
- Over Wi-Fi, `ros2 topic echo` hangs because bandwidth saturates. Use Ethernet. This is a known issue with no fix.

### 1.12 ACRE Lab (NC State) wiki, "Unitree Go2": **V**
https://github.com/ACRE-Lab-NCSU/ACRE-Wiki/wiki/Unitree-Go2 (last edited 2026-07-11)

This is a second independent lab setup. Host IP 192.168.123.99. Go2 onboard computer at 192.168.123.161 and Jetson Orin
at 192.168.123.18, both reachable over SSH as user `unitree`. A Livox Mid-360 sits at 192.168.123.20. They run ROS 2 in
Docker with `--network host` and pass `CYCLONEDDS_URI` with the interface name into the container.

Search snippets (**S**) from a Korean developer manual and a shop forum say 192.168.123.161 is the motion-control board
and that a host must not take that address.

### 1.13 Weston Robot documentation, "Go2 Robot Dog": **V**
https://westonrobot.github.io/docs-legacy/robot_user_guide/unitree/go2.html (2024, revised 2025-03-02)

This is a distributor's spec summary. Top speed is 2.5 m/s for AIR, 3.5 m/s for PRO and 3.7 m/s for EDU. The connection
modes are AP, Wi-Fi STA-L, Wi-Fi STA-T and 4G. The L1 is described as "self-developed 4D LIDAR L1". Foot force sensors
are on EDU only. It has no network or ROS content and links out to the Go2 user guide, which I did not open (**U**).

### 1.14 Intelligent Robotics Lab, go2_robot docs: **V**
https://unitree-go2-robot.github.io/humble/api_interfaces/index.html (repo:
https://github.com/Unitree-Go2-Robot/go2_robot, 334 stars, last push 2025-03-06)

This is a Humble driver that exposes the sport API as 22 ROS 2 services. Among them are `/mode`, `/body_height`,
`/euler`, `/pose`, `/speed_level`, `/switch_gait`, `/get_state`, `/set_obstacle_avoidance`, `/say` and VUI
brightness and volume. Its build guide covers both building on a PC and building on the robot. It shows one way to wrap
the request/response sport API as ROS services. (The link between the repo and the github.io site is my inference from
the GitHub Pages naming.)

### 1.15 Security: UniPwn (IEEE Spectrum) and arXiv 2509.14139: **V**
https://spectrum.ieee.org/unitree-robot-exploit (Evan Ackerman, 25 Sep 2025, updated 24 Nov 2025) and
https://arxiv.org/abs/2509.14139 (Mayoral-Vilches, Makris, Finisterre, "Cybersecurity AI: Humanoid Robots as Attack
Vectors", Sep 2025)

UniPwn is a root-level exploit through the BLE Wi-Fi provisioning interface, using hardcoded keys. It affects the Go2,
B2, G1 and H1 and was disclosed on 2025-09-20. The suggested mitigation is isolated Wi-Fi networks and Bluetooth turned
off. Unitree said on 29 September that most fixes were done, and the article does not confirm a patch. The arXiv paper
covers the G1 only, including its telemetry to external servers every 300 s, and does not test the Go2.

Separately, a closed unmerged PR on unitree_ros2 (#24, 2026) points out that DDS has no authentication. Any host on the
robot's network can publish `/lowcmd`, and the SDK does not clamp torque.

### 1.16 Simulators and ROS 2 versions the community uses (2025-2026): **V**

- **ROS 2 distros**, REP-2000 source on GitHub (ros-infrastructure/rep). The docs.ros.org and ros.org pages were behind
  an Anubis bot wall.
  - Humble: May 2022 to May 2027, Ubuntu Jammy.
  - Jazzy: May 2024 to May 2029, Ubuntu Noble.
  - Kilted: May 2025 to November 2026.
- **Gazebo/ROS pairing**, https://gazebosim.org/docs/latest/ros_installation/: Humble with Fortress is recommended and
  Humble with Harmonic is "possible, use with caution". Jazzy with Harmonic is recommended. Kilted with Ionic is
  recommended.
- **legubiao/quadruped_ros2_control** (587 stars, 2026-02-22): ros2_control controllers (Unitree guide FSM, OCS2) with a
  Go2 description.
  - The default branch is Jazzy. A humble branch also exists.
  - It runs in Gazebo Harmonic (`ros-jazzy-ros-gz`), in unitree_mujoco, on a real Go2 (added 2025-03-30) and in Isaac
    Sim (added 2025-05-20).
  - It warns that the CycloneDDS ROS 2 RMW can conflict with unitree_sdk2, and recommends switching ROS to FastDDS or
    building Cyclone the unitree_ros2 way.
- **khaledgabr77/unitree_go2_ros2** (180 stars, 2026-11-02): Go2 with CHAMP on Jazzy, Ubuntu 24.04 and Gazebo Harmonic.
  It simulates an IMU, 2D and 3D lidar, the L1 ("need some improvements"), a camera and a D455. It states that full SLAM
  and Nav2 are not done yet.
- **Zhefan-Xu/isaac-go2-ros2** (613 stars, 2025-09-23): Isaac Sim 4.5 with Isaac Lab 2.1, Humble and Ubuntu 22.04.
  Topics include `/unitree_go2/cmd_vel` and `/unitree_go2/lidar/point_cloud`. It is built for testing navigation.
- **abizovnuralem/go2_omniverse** (1,096 stars, 2026-07-01) has two tracks. Track A is Humble with Isaac Sim 2023.1.1
  and Orbit. Track B is experimental: Jazzy with Isaac Sim 5.0. It streams camera, RTX lidar and IMU to ROS 2 and
  accepts ROS 2 control.
- **isaac-sim/IsaacLab** (8,739 stars, 2026-11-27): it ships the RL tasks `Isaac-Velocity-Flat-UnitreeGo2` and
  `Isaac-Velocity-Rough-UnitreeGo2`. These are locomotion policy training tasks, not ROS 2 navigation.
- **google-deepmind/mujoco_menagerie `unitree_go2`** (4,437 stars, 2026-11-27): an MJCF model derived from Unitree's
  URDF, needing MuJoCo 2.2.2 or later, with an MJX scene.
- **fan-ziqi/rl_sar** (1,518 stars, 2026-08-16): runs RL policies in simulation and deploys them on real quadrupeds. It
  uses Foxy or Humble with Gazebo Classic and MuJoCo 3.2.7.
- **Pattern** (my inference from the dates and branches above):
  - Real-robot work still sits on Foxy (the robot's onboard computer) and Humble (Unitree's recommendation).
  - New simulation-first repos from 2025-2026 target Jazzy with Gazebo Harmonic.
  - Isaac Sim is the choice for RL and photoreal sensors.
  - MuJoCo (unitree_mujoco) is the choice for sim-to-real of low-level controllers.
  - Gazebo Classic shows up only in older or RL-deployment repos.

---

## 2. What a course like ours should take from this

The course today runs Humble, Gazebo Classic 11 and CHAMP, and Nav2 drives the robot through `/cmd_vel`, per README.md
and docs/02-prerequisites.md. The items below are the gap between that and a real Go2. They are in course order, and
each one is a concrete addition, not a rewrite.

1. **Add a "which Go2 do you have" checkpoint before any hardware chapter.**
   - EDU: Ethernet DDS access through unitree_ros2 and sdk2, plus foot force sensors.
   - AIR or PRO: WebRTC over Wi-Fi only, through go2_ros2_sdk or unitree_webrtc_connect. Lidar about 7 Hz, joint states
     1 Hz, and the firmware version decides whether an AES key is needed.
   - Record the firmware version and the lidar model (L1, or L2 on current units) as the first step, because both
     change the instructions.
2. **Add a network chapter with one tested recipe.**
   - Use a static IP on 192.168.123.0/24 (.99 per Unitree; .100 and .222 also work in labs). The robot has no DHCP.
   - Never use 192.168.123.161 (the motion board) or 192.168.123.18 (the Jetson) for the host.
   - Set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
   - Set `CYCLONEDDS_URI` with the right interface name.
   - Keep `ROS_DOMAIN_ID=0` on the robot and a different domain in simulation.
   - Run `ros2 daemon stop` then `ros2 daemon start` after changing any of these.
   - End with a `ros2 topic echo /sportmodestate` check that must pass before going further.
3. **Explain why the DDS version matters.** The robot runs CycloneDDS 0.10.2. Humble's apt Cyclone works with it
   directly. Foxy needs a source build in a terminal where ROS is not sourced. Jazzy is not yet merged upstream (PR #29).
   Since the course is on Humble, say plainly that Humble is the supported distro for the real robot and Jazzy is
   community-only for now.
4. **Write the missing `/cmd_vel` bridge as a small course node.** It is about 30 lines. It subscribes to `Twist`, calls
   SportClient Move (API 1008) on `/api/sport/request` with vx, vy and vyaw, clamps them to a conservative course limit
   (for example 0.5 m/s and 1.0 rad/s), and sends StopMove (1003) when no message arrives for 0.5 s. Unitree has not
   merged one (open PRs #17 and #21). Without it, the course's Nav2 stack cannot drive the real robot. Check the Move
   ranges and the roughly 1 s command persistence against the official sport-services page in a browser first (tag S
   today).
5. **Teach the two control levels and the rule that you never mix them.**
   - High-level (sport mode) is where all navigation belongs: Move, StandUp, StandDown, BalanceStand, Damp.
   - Low-level (`/lowcmd`: per-joint q, dq, tau, kp and kd with a CRC) only after switching off the motion service
     (MotionSwitcherClient ReleaseMode or the app). Only with the robot hung up or lying down. Only with low gains
     (Unitree's examples use kp 5 to 10 and kd 1), and starting from the passive values (PosStopF, kp 0).
   - Point out that `sportmodestate` stops publishing on the real robot once sport mode is off, while unitree_mujoco
     keeps publishing it.
6. **Add a short safety protocol page.**
   - Clear area and no people within reach.
   - A hand on the Unitree remote at all times. Damp (1001) is the emergency stop and the course's E-stop key should map
     to it.
   - StandDown before power-off.
   - No flips or "wallow" moves.
   - Built-in obstacle avoidance is a separate switch (SwitchAvoidMode, ObstaclesAvoidClient). Do not assume it is on.
   - Follow the robot on the first autonomous runs.
   - Security: turn Bluetooth off, use an isolated network (UniPwn), and remember that anyone on the robot's network can
     publish `/lowcmd`.
7. **Add a "real sensors are not the sim sensors" chapter alongside chapter 08.**
   - The internal L1 publishes `utlidar/cloud` and `utlidar/imu` in frame `utlidar_lidar`, mounted inverted and tilted.
     CMU rotates it by pitch 2.878 rad.
   - Its built-in IMU needs a one-time calibration.
   - The camera comes as an H.264 stream that must be republished as `sensor_msgs/Image`.
   - The robot's own state estimate (`sportmodestate.position` and `velocity`) is available but has no tf.
   - Learners must publish their own `odom` to `base_link` and lidar static transforms, and check them in RViz against
     the frame definition in unilidar_sdk.
8. **Make time and QoS explicit.**
   - Robot timestamps are not host time. CMU computes a one-off offset, which is the simplest fix, but it is only valid
     if the clock does not drift during a run.
   - Subscribe to telemetry with sensor-data (best-effort) QoS, per PR #29. If nothing arrives, check QoS before
     anything else (Southampton troubleshooting).
   - Wi-Fi saturates on point clouds, so do mapping on Ethernet or on the onboard computer.
9. **Use bags as the bridge between simulation and hardware.**
   - Record `ros2 bag record /utlidar/cloud /utlidar/imu` (plus `/sportmodestate`) on the robot.
   - Replay offline through the course's estimator and mapping stack with the robot disconnected, so live data cannot
     leak in.
   - This lets a learner test chapters 09 and 10 on real data before the robot walks under autonomy.
10. **Choose the next simulator deliberately.**
    - Gazebo Classic 11 is end-of-life. For a Humble-first course, Gazebo Fortress is the recommended pair and Harmonic is
      possible.
    - A move to Jazzy pairs with Harmonic (legubiao and khaledgabr77 already run the Go2 there), but the real-robot
      chapter would then depend on the unmerged unitree_ros2 Jazzy work.
    - Add unitree_mujoco as an optional bonus chapter. It is the only simulator that speaks the robot's own DDS messages,
      so the same binary runs in simulation and on hardware.
    - Skip Isaac Sim and Isaac Lab for this course except as a pointer for RL locomotion. They need heavy GPU setup and
      serve a different goal.
11. **Use reference stacks instead of new code.** go2_ros2_sdk is the real-robot counterpart to the course's
    slam_toolbox plus Nav2 setup. autonomy_stack_go2 is a lidar-inertial SLAM and terrain-aware avoidance stack on the
    L1. A closing chapter can map the course's packages onto these two rather than adding new features.
12. **Open items to verify by hand before publishing:**
    - Read the official Unitree pages (Quick_start, sports_services, Basic_services, ROS2_service) in a browser for:
      - Move ranges and timeout.
      - The frame of `sportmodestate.position`.
      - The `lf/` publish rates.
      - Official safety prerequisites.
    - Check whether PR #29 (Jazzy) or a `/cmd_vel` bridge has been merged.
    - Confirm which lidar (L1 or L2) a learner's unit ships with.
