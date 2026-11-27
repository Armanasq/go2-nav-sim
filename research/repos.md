# GitHub repositories and tutorials for quadrupeds in ROS 2

Research angle: GitHub tutorials and repositories that teach or implement quadruped robots (Unitree Go2 in particular)
with ROS 2, plus ROS 1 projects where they are still the reference. Every repository below was opened on 2026-11-28
with the `gh` CLI: the README was read, the top-level tree and (where present) the docs tree listed, and stars, last
push date and archive status taken from the GitHub API on that date. Docs sites were opened with a web fetch. Items
marked **unverified** could not be opened or read in full.

Our course, for comparison: `go2-nav-sim`, ROS 2 Humble, Gazebo Classic 11, Go2 URDF from `anujjain-dev/unitree-go2-ros2`
walking with the CHAMP gait controller, own lidar-inertial odometry + EKF, AMCL, slam_toolbox and Nav2, simulation only.
It has no real-robot bridge, no command-arbitration or e-stop layer, no Docker image, and one gait controller.

## 1. Official Unitree repositories

**unitreerobotics/unitree_ros2** — https://github.com/unitreerobotics/unitree_ros2 — 905 stars, last push 2026-07-02.
The ROS 2 interface to the real Go2/B2/H1/G1. Go2 speaks CycloneDDS natively, so the package is mostly message
definitions (`unitree_go`, `unitree_api`) plus a `setup.sh` that sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and binds
`CYCLONEDDS_URI` to the Ethernet interface (robot network 192.168.123.x, PC at .99). A learner can read
`/sportmodestate` (position, velocity, gait type, foot positions, `range_obstacle`), `/lowstate` (20 motor states, IMU,
battery, foot force), `/wirelesscontroller`, and the `utlidar/cloud` point cloud; and command the robot either through
high-level sport mode (request/response on `/api/sport/request`, built with a `SportClient` helper) or low-level
`/lowcmd` (per-joint q, dq, tau, kp, kd). Structure: one long README walkthrough plus C++ examples
(`read_motion_state`, `read_low_state`, `go2_sport_client`, `go2_stand_example`, `low_level_ctrl`, `record_bag`), a
`.devcontainer`, and `setup_local.sh` for loopback use. What it has that our course lacks: the real topic names, the
DDS/network setup, and the two command levels. Weak: no simulator of its own, no TF/URDF, no Nav2, Foxy-era
instructions (rebuild CycloneDDS 0.10.x, comment out `source` in `.bashrc`), examples in C++ only, and the field
documentation points to support.unitree.com (**unverified**: the page is JavaScript-rendered and returned only its
title).

**unitreerobotics/unitree_sdk2** — https://github.com/unitreerobotics/unitree_sdk2 — 1,462 stars, last push 2026-11-19.
The C++ SDK (CMake, Ubuntu 20.04, gcc 9.4, x86_64 and aarch64) that `unitree_ros2`, `unitree_mujoco` and most
community stacks build on. README is a build/install note only; all usage is in `example/` and the vendor docs site.
Useful to a learner only as the layer beneath ROS 2. Weak: almost no in-repo documentation.

**unitreerobotics/unitree_sdk2_python** — https://github.com/unitreerobotics/unitree_sdk2_python — 852 stars, last push
2026-09-21. Python bindings with the same API. The README is a better beginner entry than the C++ one: hello-world DDS
publisher/subscriber, read high-level state, a `sportmode_test.py` with `StandUpDown`, `VelocityMove`,
`BalanceAttitude`, `TrajectoryFollow`, low-level motor control (with the warning to switch off sport mode first and
conservative `kp=10, kd=1`), wireless controller, front camera via OpenCV, obstacle-avoidance toggle. Each example takes
the network interface as its only argument. Weak: no ROS 2; the cyclonedds 0.10.2 build is the usual failure point.

**unitreerobotics/unitree_mujoco** — https://github.com/unitreerobotics/unitree_mujoco — 1,316 stars, last push
2026-09-07, 69 open issues. A MuJoCo simulator (C++ and Python) that publishes and accepts the same DDS messages as the
real robot (`LowCmd`, `LowState`, `SportModeState`), so code written for `unitree_sdk2`, `unitree_sdk2_python` or
`unitree_ros2` runs unchanged against it. Config sets `domain_id: 1` (real robot is 0) and `interface: "lo"`, maps an
Xbox/Switch gamepad to the wireless-controller topic, and includes a terrain generator. The "Sim to Real" section runs a
`stand_go2` example first in sim, then on the robot by passing the NIC name. What it does that our course does not: a
sim that is wire-compatible with the real robot. Weak: low-level only ("current version only supports low-level
development"); no built-in walking controller, so nothing walks until you supply one; no lidar or camera.

**unitreerobotics/unitree_ros** — https://github.com/unitreerobotics/unitree_ros — 1,660 stars, last push 2026-11-27.
ROS 1 repository, but the source of `robots/go2_description` (and descriptions for Go1, B2, G1, H1 and others) that most
Go2 ROS 2 projects, including ours via `anujjain-dev/unitree-go2-ros2`, copy. Learners use it only for the URDF/meshes.

**unitreerobotics/unitree_guide** — https://github.com/unitreerobotics/unitree_guide — 644 stars, last push 2024-07-22.
ROS 1 Melodic, Gazebo. A basic model-based quadruped controller with a keyboard-driven finite-state machine (Passive →
FixedStand → trotting), written as the companion code to Unitree's book 《四足机器人控制算法--建模、控制与实践》
("Quadruped robot control algorithms: modelling, control and practice"). The README says it is for beginners and that
MPC or tuning is needed for better performance. Book table of contents: **unverified** (not opened). Its controller has
been ported to ROS 2 inside `legubiao/quadruped_ros2_control` (below).

**unitreerobotics/unitree_rl_gym** — https://github.com/unitreerobotics/unitree_rl_gym — 3,692 stars, last push
2025-07-25, 63 open issues. legged_gym-style RL for Go2, G1, H1, H1_2 on Isaac Gym, framed as
`Train → Play → Sim2Sim (MuJoCo) → Sim2Real`, with `--task=go2` among the training tasks and policy export to TorchScript.
Weak for a Go2 learner: `deploy/deploy_mujoco/configs` and `deploy/deploy_real/configs` contain only `g1.yaml`,
`h1.yaml`, `h1_2.yaml`, so the sim-to-sim and sim-to-real steps are not provided for Go2. Isaac Gym is deprecated.

**unitreerobotics/unitree_rl_lab** — https://github.com/unitreerobotics/unitree_rl_lab — 1,473 stars, last push
2026-05-25. The Isaac Lab successor (Go2, H1, G1-29dof), with a wrapper script to list and run tasks and robot assets
loaded from `unitree_ros` URDFs (Isaac Sim ≥ 5.0) or USD files. No ROS 2 navigation content.

## 2. Go2 ROS 2 community stacks (real robot)

**abizovnuralem/go2_ros2_sdk** — https://github.com/abizovnuralem/go2_ros2_sdk — 1,085 stars, last push 2026-07-13, 26
open issues. Unofficial ROS 2 driver for Go2 AIR/PRO/EDU over WebRTC (Wi-Fi, so it also works on non-EDU models) or
CycloneDDS (Ethernet). One `robot.launch.py` brings up robot_state_publisher, camera, lidar-to-pointcloud,
pointcloud_to_laserscan, joystick teleop, `twist_mux`, Foxglove bridge, slam_toolbox and Nav2. The README is the closest
thing to a hands-on real-robot mapping tutorial: mark a "dock" with tape so the start pose is repeatable, save a map
from the slam_toolbox RViz plugin, deserialize it next session, then send Nav2 goals; and it states that "virtually all
fault behaviors" come from a wrong map or wrong initial pose. Also COCO object detection, multi-robot, Docker compose,
WSL2 joystick setup. What it does that our course does not: real-robot SLAM/Nav2 workflow, `twist_mux` priority between
teleop and Nav2, Docker. Weak: no simulator; firmware ≥ 1.1.15 needs a per-device AES key fetched with the user's
Unitree account; dependencies via `pip install -r requirements.txt` (open3d breaks on Python 3.12); parameters are not
explained; the author notes a recent large refactor.

**Unitree-Go2-Robot/go2_robot** (was IntelligentRoboticsLabs/go2_robot) — https://github.com/Unitree-Go2-Robot/go2_robot —
334 stars, last push 2025-03-06. Universidad Rey Juan Carlos lab driver on top of `unitree_ros2`: odom, point cloud,
joint states, `cmd_vel`, and one ROS 2 service per sport-mode function (`/mode` with damp, stand_up, sit, hello, ...;
`/body_height`, `/euler`, `/foot_raise_height`, `/switch_gait`, `/speed_level`), with the legal ranges listed; Hesai
lidar and RealSense D435i add-ons; dependencies through `vcs import`. Useful as a clean mapping of the vendor API to
ROS 2 idioms. Weak: the checklist marks SLAM and Nav2 done, but the SLAM and NAVIGATION sections say "In the future, work
in progress"; no Gazebo simulation; no hardware interface.

**jizhang-cmu/autonomy_stack_go2** — https://github.com/jizhang-cmu/autonomy_stack_go2 — 578 stars, last push 2026-02-18,
28 open issues. CMU's full autonomy stack ported to Go2 (ROS 2 Foxy/Humble): SLAM from `point_lio_unilidar` on the
built-in L1 lidar, terrain traversability analysis, a local planner with smart-joystick / waypoint / manual modes, and
a visibility-graph route planner; Unity environment models for simulation, real-robot setup on the onboard computer or
an external PC. Not a Nav2 stack. Its "Notes" section is the most useful list of real-Go2 pitfalls found in this search:
L1 lidar IMU must be calibrated once per robot; Humble on an external PC showed > 1 s data delay; the L1 lidar is noisy
and the terrain analysis cannot see obstacles lower than about 0.3 m; SLAM drift causes the robot to get stuck; camera
timestamps are not synchronised with lidar/IMU and recording images to a bag loses frames. Weak: Unity models from a
Google Drive link, Foxy-centred, its own planner rather than Nav2, little explanation of the internals.

**eppl-erau-db/go2_rl_ws** — https://github.com/eppl-erau-db/go2_rl_ws — 89 stars, last push 2026-09-11. ROS 2 Humble
workspace that runs ONNX locomotion and 2D navigation RL policies on a real Go2 through `unitree_ros2` low-level
commands. Points worth copying: it explains which observations were removed from the trained policy (height scan, base
linear velocity) because they are not available in low-level mode, and the deployment instructions say the robot must be
lying down with sport mode off and preferably tethered. Weak: hard-coded NIC names in launch files, nav policy without
obstacles.

**legion1581/unitree_webrtc_connect** — https://github.com/legion1581/unitree_webrtc_connect — 428 stars, last push
2026-11-25. Python WebRTC driver for Go2/G1 used by `go2_ros2_sdk`; relevant only if the course ever targets a non-EDU
Go2 (README read for scope only).

## 3. Go2 in Gazebo with ROS 2 (the layer our course uses)

**chvmp/champ** — https://github.com/chvmp/champ — 2,343 stars, last push 2024-07-04, 54 open issues. The CHAMP gait
controller (MIT Cheetah I hierarchical controller) that our course uses. `master` is ROS 1 (Kinetic/Melodic) and has the
clearest quick start in this list: walk in RViz, teleop, SLAM with gmapping + move_base, save map, AMCL navigation, then
"run your own robot" via `champ_setup_assistant`, gait parameters, hardware drivers, and microcontroller firmware. The
`ros2` branch (Humble, slam_toolbox + Nav2) was last committed 2023-02-24. Wiki: two pages; "Hardware Integration" failed
to load (**unverified**). Weak: unmaintained ROS 2 branch, open-loop gait whose odometry needs an "odometry scaler", no
state estimation beyond leg kinematics + IMU.

**anujjain-dev/unitree-go2-ros2** — https://github.com/anujjain-dev/unitree-go2-ros2 — 299 stars, last push 2024-12-06.
Go2 description + CHAMP config for Humble and Gazebo Classic; our course builds on it. Learner gets Gazebo + RViz,
keyboard teleop, IMU, 2D Hokuyo or Velodyne, and a table of gait parameters (`gait.yaml`). Its checklist marks Gazebo
SLAM and Nav2 as not done, which is the gap our course fills. Weak: no maintenance since 2024, no estimation, no
navigation.

**khaledgabr77/unitree_go2_ros2** — https://github.com/khaledgabr77/unitree_go2_ros2 — 180 stars, last push 2026-11-02.
Same CHAMP approach ported to ROS 2 Jazzy and Gazebo Harmonic (`ros2_control` effort interface by default, position
interface optional). Sensors: IMU, Hokuyo, Velodyne VLP-16, Unitree L1 model, mono camera, RealSense D455, GPS with
documented noise; each sensor can be disabled by a launch argument to save CPU. A fork (RobInLabUJI/unitree_go2_ros2_jazzy)
exists; a further fork adds per-foot contact topics. Weak: SLAM and Nav2 listed as "coming soon"; GPS covariance is
zero after the Gazebo bridge, which the README itself flags.

**legubiao/quadruped_ros2_control** — https://github.com/legubiao/quadruped_ros2_control — 587 stars, last push
2026-02-22. `ros2_control` controllers for quadrupeds (Jazzy default, `humble` branch available): `leg_pd_controller`,
`unitree_guide_controller` (ROS 2 port of unitree_guide), `ocs2_quadruped_controller` (MPC + WBC, adapted from
legged_control), and `rl_quadruped_controller`. Runs in Gazebo Harmonic (`gz_quadruped_playground` with lidar/depth
worlds and a SLAM-test section), in `unitree_mujoco`, in Isaac Sim, and on a real Go2 through a hardware interface. Same
keyboard command node for every controller. What it does that our course does not: lets a learner swap the walking
controller behind the same command interface and run the same code on sim and robot. Weak: README is a launch list
with no explanation of the controllers; warns that CycloneDDS RMW conflicts with `unitree_sdk2` (recommends FastDDS).

**Zhefan-Xu/isaac-go2-ros2** — https://github.com/Zhefan-Xu/isaac-go2-ros2 — 613 stars, last push 2025-09-23. Go2 in
Isaac Sim 4.5 / Isaac Lab 2.1 with a pre-trained RL walking policy, publishing ROS 2 Humble topics: `cmd_vel`, RGB, depth,
semantic segmentation, camera info, lidar point cloud, odom, pose. Seven environments (warehouse variants, sparse/medium/
dense obstacle fields), multi-robot via `num_envs`, demo with the NavRL navigation policy. What it does that our course
does not: photoreal sensors, RL gait, semantic camera. Weak: needs an RTX GPU and an Isaac install; odometry is
simulator-provided; no Nav2 example.

**abizovnuralem/go2_omniverse** — https://github.com/abizovnuralem/go2_omniverse — 1,096 stars, last push 2026-07-01.
Go2/G1 in Isaac Lab with ROS 2. The current README describes a fork-style addition: a "twinbot" mode that replays the real
robot's `/lowstate` into the sim and an Isaac Sim 5.x / Jazzy port. Not a teaching resource; listed because it is often
recommended.

## 4. Model-based control frameworks (ROS 1 references)

**leggedrobotics/ocs2** — https://github.com/leggedrobotics/ocs2 — 1,546 stars, last push 2026-09-29. Switched-system
optimal-control toolbox (SLQ, iLQR, SQP, SLP, IPM; Pinocchio, CppAD). `main` is ROS 1; a `ros2` branch exists (last
commit 2026-09-29). Docs site https://leggedrobotics.github.io/ocs2/ (opened): the "Legged Robot" example runs ANYmal with
a user-set, switchable gait, SRBD or full centroidal dynamics, friction-cone and swing-foot constraints, launched with
`roslaunch ocs2_legged_robot_ros legged_robot_ddp.launch` or `..._sqp.launch`. Value for a course: the clearest written
description of what an MPC locomotion controller optimises. Weak: huge monorepo; Go2 is not an example.

**qiayuanl/legged_control** — https://github.com/qiayuanl/legged_control — 1,858 stars, archived, last push 2025-02-13.
NMPC + WBC + state estimation on OCS2 and ros_control, ROS 1, Unitree A1/Aliengo. README states it is no longer
supported. Still the reference that the ROS 2 OCS2 controllers (legubiao) derive from. Its "cheater" controller (uses
ground-truth state, banned on hardware) is a pattern worth naming in a course.

**robomechanics/quad-sdk** — https://github.com/robomechanics/quad-sdk — 975 stars, last push 2026-11-25. CMU
Robomechanics full stack: global body planner, local footstep planner, NMPC, EKF, logging, Gazebo. `main` README is ROS 1
Melodic/Noetic (last main commit 2024-04-25), but there are about 30 `devel_ros2_*` branches, including
`devel_ros2_go2_learned` (commit 2026-09-03). Wiki (opened, titles only; page bodies **unverified**) has "Using the
Software (ROS2)", "Adding a New Type of Robot (ROS2)", "Running UnitreeGo2", "Writing and Executing Your Own Controller"
and "Creating Custom Terrain Map Files". Weak: ROS 2 work lives on development branches, not documented in the README.

**iit-DLSLab/Quadruped-PyMPC** — https://github.com/iit-DLSLab/Quadruped-PyMPC — 540 stars, last push 2026-11-11.
Single-rigid-body MPC in Python: gradient-based (acados, < 5 ms on an i7) and sampling-based (JAX, MPPI and variants, GPU).
Supports Go2 and others; MuJoCo simulation; real deployment through separate `muse` (state estimation) and
`unitree-ros2-dls` repos; a `ros2/` folder. The most readable MPC code found here. Weak: research code, install via pixi.

## 5. Learned locomotion (RL)

**leggedrobotics/legged_gym** — https://github.com/leggedrobotics/legged_gym — 3,202 stars, last push 2025-05-29. The
Isaac Gym rough-terrain RL environments (Rudin et al., arXiv 2109.11978) that most Go2 RL repos fork. README explains the
code pattern: env + config classes, reward terms enabled by non-zero scales, task registry, domain randomisation, pushes,
noisy observations. Announcement (2024-01-09): migrated to Isaac Lab; this repo is no longer the active line.

**isaac-sim/IsaacLab** — https://github.com/isaac-sim/IsaacLab — 8,739 stars, last push 2026-11-27. Source contains
`Isaac-Velocity-Flat-UnitreeGo2` and `Isaac-Velocity-Rough-UnitreeGo2` with RSL-RL and skrl configs (read from
`core/velocity/config/go2/__init__.py`). The only quadruped navigation task in the tree is ANYmal-C
(`contrib/navigation/config/anymal_c`). The environments docs page moved and the fetched copy was truncated before the
task list (**unverified** for the docs page; task IDs verified from source).

**Improbable-AI/walk-these-ways** — https://github.com/Improbable-AI/walk-these-ways — 1,475 stars, last push 2024-06-16.
Go1 sim-to-real starter kit (Margolis and Agrawal): gait-conditioned policy, PPO + Isaac Gym + domain randomisation, LCM
deployment. **Teddy-Liao/walk-these-ways-go2** — https://github.com/Teddy-Liao/walk-these-ways-go2 — 635 stars, last push
2026-07-14 — ports it to Go2 by replacing the UDP SDK with `unitree_sdk2` behind an LCM bridge; ships a pre-trained Go2
policy; deployment from a PC only. Its deploy steps include the warning that starting the bridge turns off sport mode
and that the robot must be hung up or lying down. Weak: Isaac Gym, PyTorch 1.10/CUDA 11.3, no ROS.

**fan-ziqi/rl_sar** — https://github.com/fan-ziqi/rl_sar — 1,518 stars, last push 2026-08-16. Runs trained RL policies
(libtorch or ONNX) in Gazebo or MuJoCo and on real robots, with ROS Noetic or ROS 2 Foxy/Humble; Go2 has pre-trained
policies (himloco, robot_lab) marked working in Gazebo, MuJoCo and real. Training is in the companion
**fan-ziqi/robot_lab** (https://github.com/fan-ziqi/robot_lab, 1,992 stars, last push 2026-10-20, Isaac Lab extension).
Value: the shortest path from "CHAMP in Gazebo" to "learned gait in Gazebo" under ROS 2. Weak: README carries a
liability disclaimer for hardware use; Python version suspended.

**google-deepmind/mujoco_playground** — https://github.com/google-deepmind/mujoco_playground — 2,353 stars, last push
2026-11-27. Current release targets manipulation, whole-body tracking and agentic tasks (robots: g1, so101, trossen,
yam). The original RSS 2025 JAX/MJX locomotion environments are kept under `rss2025/` and include Go1 (joystick,
getup, handstand), Barkour, Berkeley Humanoid, G1, H1; there is no Go2 environment. Colab quickstart. Not ROS.

**google-deepmind/mujoco_menagerie** — https://github.com/google-deepmind/mujoco_menagerie — 4,437 stars, last push
2026-11-27. `unitree_go2/` has `go2.xml`, `go2_mjx.xml` and scenes; our course already uses it for figures.

## 6. Tutorial-format resources

**ztl3106742440-hub/go2-tutorial** — https://github.com/ztl3106742440-hub/go2-tutorial (site
https://ztl3106742440-hub.github.io/go2-tutorial/, opened) — 27 stars, created 2026-04-21, last push 2026-09-16. Chinese
MkDocs book for Go2 + ROS 2, real-robot first: overview, hardware, environment, a "real-robot safety red lines" chapter,
then install, Go2 message interfaces, keyboard node, Twist bridge, visualisation, driver package, topics/services/actions,
lidar, 2D SLAM, real-robot Nav2 2D baseline, simulation set-up, 3D SLAM + Nav2 in simulation, external MID-360 lidar,
voice, vision/VLM, task orchestration, RL outlook. Each chapter opens with "what you will learn" and shows screenshots of
the expected result. The Nav2 chapter uses `twist_mux` to merge navigation, recovery and manual takeover into one
`/cmd_vel`, and has a table of the 2D-navigation assumptions that break on a quadruped (flat ground, obstacles in the scan
plane, smooth odometry, circular footprint). The safety chapter keeps learners on the high-level API and treats `/lowcmd`
as a hazard. Weak: small project, Chinese only, code split across sibling repos, no measured results.

**ros-navigation/navigation2** and **ros-navigation/docs.nav2.org** — https://github.com/ros-navigation/navigation2
(5,120 stars, last push 2026-11-27) and https://docs.nav2.org/ (source repo 193 stars, last push 2026-11-24). Not
legged-specific; listed because every Go2 stack above that navigates uses Nav2 or replaces it, and none documents
quadruped-specific Nav2 parameters.

Also checked and not used: `hku-mars/FAST_LIO` (5,427 stars, ROS 1) and `Ericsii/FAST_LIO_ROS2` (728 stars, last push
2025-11-17) as lidar-inertial odometry options; `leggedrobotics/elevation_mapping_cupy` (1,182 stars, last push 2026-11-27)
for rough-terrain maps; `leggedrobotics/rsl_rl` (3,167 stars) as the PPO library under legged_gym/Isaac Lab;
`dimensionalOS/dimos` (4,782 stars, natural-language agent layer, out of scope). READMEs not read in depth.

## What a course like ours should take from this

1. **Add a "from simulation to the real Go2" chapter that is a mapping, not a driver.** One table: our `/cmd_vel` →
   `unitree_ros2` `/api/sport/request` (SportClient `Move`) or `go2_ros2_sdk` `cmd_vel`; our `/lidar/points` →
   `utlidar/cloud`; our `/imu/data` → `lowstate.imu_state` / `utlidar/imu`; our joint states → `lowstate.motor_state`.
   Plus the network recipe from `unitree_ros2` (`setup.sh`, `CYCLONEDDS_URI`, 192.168.123.x). No new code needed.
2. **Put a command arbitration layer in front of the robot.** `twist_mux` with teleop above Nav2 (as `go2_ros2_sdk` and
   go2-tutorial do), plus Nav2's collision monitor and a command timeout. Our stack sends Nav2 output straight to CHAMP.
3. **Write a one-page safety checklist and reference it before any hardware step.** Collected rules: high-level API
   first, `/lowcmd` only with the robot hung or lying down and sport mode off, remote control in hand, low gains first
   (`kp=10, kd=1` in the SDK example), clear area, cables away from the legs.
4. **State the 2D-Nav2-on-a-legged-robot assumptions explicitly.** We already document the footprint compromise; add
   obstacles outside the scan plane, no stairs/steps handling, gait-induced IMU noise, and the CMU field note that the
   L1 lidar does not resolve obstacles under about 0.3 m. Each as one sentence with the consequence.
5. **Add a "what changes on hardware" list from the CMU Go2 notes**: per-robot L1 IMU calibration, DDS delay on an
   external Humble PC, unsynchronised camera timestamps, frame loss when bagging images, SLAM drift recovery (clear the
   map). These are failures our simulation cannot show; a short replay exercise on a real Go2 bag of
   `/utlidar/cloud` + `/utlidar/imu`, if a public one is found, would let learners run our odometry on real data.
6. **Name the Jazzy/Gazebo Harmonic port target concretely.** `khaledgabr77/unitree_go2_ros2` is the same CHAMP + Go2
   approach, actively maintained (push 2026-11-02), with per-sensor disable flags. Point chapter 2's roadmap note at it
   instead of a generic "porting is planned".
7. **Offer one optional "beyond CHAMP" section, not a new stack.** Show that the walking controller sits behind
   `cmd_vel` and can be swapped: `legubiao/quadruped_ros2_control` (unitree_guide, OCS2 MPC, RL in Gazebo Harmonic with
   the same keyboard node) or `rl_sar` (pre-trained Go2 policy in Gazebo under ROS 2). One paragraph and a launch line
   each; let the learner rerun our A→B mission and compare odometry error.
8. **Use `unitree_mujoco` as the bridge exercise for real-robot code.** It accepts the real DDS messages on domain 1 /
   loopback, so a learner can test `unitree_sdk2_python` high-level calls without a robot. Note its limit: no built-in
   gait in low-level mode and no lidar, so it does not replace Gazebo for navigation.
9. **Copy go2-tutorial's chapter scaffolding**: a "you will learn" list at the top and a screenshot or printed output
   of the expected result at each step. Our chapters have exercises only in chapters 4 and 5; add one checkable exercise
   per chapter (for example, "`ros2 topic hz /odom` shows about N Hz").
10. **Ship a container.** `unitree_ros2` has a `.devcontainer` and `go2_ros2_sdk` a docker-compose; we have neither. A
    single Humble + Gazebo Classic image would remove most of chapter 2's install failures. Also mention Foxglove as an
    alternative viewer to RViz for remote/headless runs, as `go2_ros2_sdk` does.

Things to avoid copying: launch-everything files without parameter explanations (`go2_ros2_sdk`), checklists that
claim features the README later says are unfinished (`go2_robot`), RL pipelines whose Go2 deploy step is missing
(`unitree_rl_gym` has sim-to-sim and real configs only for G1/H1/H1_2), and dependence on archived ROS 1 frameworks
(`legged_control`) for a beginner course.
