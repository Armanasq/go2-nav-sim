# Gap plan: what to add, cut and read

Compares the five `research/` notes with `README.md`, `docs/00`-`12`, `projects/p01`-`p03`, `modules/` and `roadmap.json`. Every source was re-opened on 2026-11-28: repositories with `gh api` (stars and last push are in the notes), web pages by fetching them and checking the title. docs.ros.org blocks scripts, so ROS tutorials are cited by their file in `ros2/ros2_documentation` (humble). Tags: **V** opened, **S** snippet only, **U** not opened.

## 1. What others teach that this course does not

| # | Gap here | Shown by (V) |
|---|---|---|
| 1 | No RViz workflow and no RViz config; Gazebo runs headless. | Nav2 setup guide (each page ends with an RViz check) |
| 2 | The URDF is only read, never edited. | ROS URDF tutorials; Articulated Robotics |
| 3 | Launch arguments and YAML overrides are never taught. | ROS `Using-Substitutions` |
| 4 | No command arbitration: the p02 node guards teleop only, and Nav2 writes `/cmd_vel` directly. | `go2_ros2_sdk` and go2-tutorial (`twist_mux`); `twist_mux` Bool locks; Nav2 Collision Monitor |
| 5 | `/body_pose` is listed but never commanded. | `go2_robot` (`/body_height`, `/euler`); upstream `champ_teleop.py` (roll ±0.35, pitch ±0.17, yaw ±0.44 rad) |
| 6 | No reactive behaviour before Nav2. | Rico ch 3, `br2_fsm_bumpgo_py` |
| 7 | No behaviour tree; Nav2's BT stays a black box. | Rico ch 6, `br2_bt_patrolling`; Nav2 Groot2 |
| 8 | The learner never makes, saves and reuses a map; it comes from a generator. | CHAMP README order; `go2_ros2_sdk` README; Nav2 "Navigating while Mapping" |
| 9 | The learner never tunes Nav2. The guide lists Legged under Smac Hybrid-A*, MPPI and RPP, and allows `robot_radius` only with holonomic planners such as NavFn; our choice is valid but unexplained. | Nav2 Tuning Guide; footprint page |
| 10 | No list of 2D-Nav2 assumptions that break on legs. | go2-tutorial; `autonomy_stack_go2` (L1 misses obstacles under ~0.3 m) |
| 11 | No ROS-level test, only pure-Python unit tests. | ROS `Testing/Integration` (launch_testing) |
| 12 | Exercises only in ch 4-5; no Goal/Time header. | ROS docs format; go2-tutorial; CS 123 deliverables |
| 13 | No container. | `unitree_ros2` `.devcontainer`; `go2_ros2_sdk` `docker/` |
| 14 | No path to a real Go2. | `unitree_ros2` (no `/cmd_vel`, no tf); `go2_ros2_sdk`; Southampton guides; `autonomy_stack_go2` |
| 15 | Leg odometry is rejected, not explained. | Bloesch et al., RSS 2012 |

## 2. What this course already does better

- Every run is scored against ground truth (ATE, RPE, AMCL RMSE, stopping time). go2-tutorial, `go2_ros2_sdk` and `go2_robot` report no measured results.
- Failures are documented with cause and fix (ch 9, 12); `go2_robot` claims features its README calls unfinished.
- Own lidar-inertial odometry with ROS-free tests; a measured dead-man timeout (p02); gait parameters traced to CHAMP source.
- Pinned, patched upstream; modules checked against Pinocchio and MuJoCo.

## 3. Plan

Every chapter gets a Goal / Time / Prerequisites / You will learn header, steps that each end with a visible check, and one exercise with its answer. P1 = largest gain in independence for least effort; P3 = optional. Hours are my estimates for a beginner: about 45 h in total, or about 43.5 h without the P3 item.

### A. Run it and see it (8.5 h)

- **Ch 0** (0.5 h): add the hour table; mark core vs deep dive.
- **Ch 2+3 merged** (2 h, P2)
  - Native install on 22.04, versions checked.
  - **Dev container** (new): runs on any host, with GUI forwarding for RViz.
  - First mission: read the result line of `run_mission.sh`.
- **Ch 4 ROS 2** (4 h, P1)
  - Graph, CLI, QoS, sim time: inspect a running system; explain a silent echo.
  - Write, build and test a node (existing).
  - **Launch arguments and YAML** (new): expose `world`, `params_file`, `use_sim_time`; override one Nav2 value; confirm with `ros2 param get/dump`.
  - Bags: record and replay.
  - *Cut:* executors to a five-line note (full text to m18); lifecycle to its two commands.
- **P01 First steps** (2 h, P1). Add **RViz**: see what the robot sees. Ship one `rviz/go2.rviz` (model, TF, scan, Best Effort cloud, map, costmaps, plan, footprint) that every later chapter reuses.

### B. The robot (8 h)

- **Ch 1** (1.5 h): one figure placing CHAMP (gait plus impedance) beside convex MPC, WBIC and an RL policy, all behind `cmd_vel`.
- **Ch 5 TF2** (2.5 h, P2)
  - REP-103/105 and `view_frames`: read the tree.
  - Lookups in time: diagnose an `ExtrapolationException`.
  - **Broken tree** (new): find one wrong static transform and one missing frame with `tf2_echo`, `tf2_monitor`, `view_frames`.
  - *Move to m00:* "Compose transforms by hand" and "missing tilt cost".
- **Ch 6 Import** (2 h, P1)
  - **Inspect in RViz** (new): move a leg with `joint_state_publisher_gui` and read its joint limits.
  - **Visual vs collision** (new): see what physics uses.
  - **Add a sensor link** (new): add it in xacro and confirm it in `view_frames`.
  - ros2_control and CHAMP maps: which names must match.
- **Ch 7 Walk** (2 h, P2)
  - Velocity limits (existing).
  - **Body posture** (new): publish `/body_pose` height, roll and pitch within the `champ_teleop` ranges. Check the trunk height on `/odom_gt`.
  - Gait YAML: edit, restart, measure.
  - *Move to m09:* "How the trunk moves", and the stroke and Bézier derivations.

### C. Drive it safely (7 h)

- **P02 Teleop and safety** (3 h, P1): keep steps 1-8.
  - **`twist_mux`** (new): teleop (100) over Nav2 (10), e-stop as a Bool lock. Check: a keypress overrides a live goal; the lock stops the robot.
  - Gamepad (optional): `joy_node` + `teleop_twist_joy` with a deadman button.
- **P03 Shapes** (2 h): keep.
- **P04 React to obstacles** (new, 2 h, P2)
  - A state machine on `/scan` (walk; turn when the nearest return is under *d*): first autonomy without Nav2. ROS-free unit test; score from `/odom_gt`.

### D. Know where it is (6 h)

- **Ch 8 Sensors** (2 h)
  - An RViz check for each sensor.
  - A sim-vs-real table: `utlidar/cloud` comes inverted in `utlidar_lidar`, the camera is H.264, and there is no tf.
- **Ch 9 Estimation** (3 h, P2)
  - **Leg odometry as a model** (new, half page): base velocity from the stance foot; why it fails here (no contacts); one plot vs ground truth.
  - Lidar odometry, shortened. The ICP derivation moves to m11.
  - **REP-103 sign test** (new, pytest on a bag): CCW turn raises yaw, forward walk raises x, only the EKF publishes `odom` → base.
  - Score against ground truth (existing).
- **Ch 11** (1 h): "Numbers" into ch 9; "Video" to an appendix.

### E. Map and navigate (8.5 h)

- **Ch 10A Mapping** (2 h, P1): a map the learner made.
  - Teleop the robot under slam_toolbox.
  - Save with `map_saver_cli`, and compare it with Serialise.
  - Reload it in RViz with Transient Local QoS.
- **Ch 10B Localise and navigate** (2 h, P1): Nav2 without ground truth.
  - AMCL on the learner's own map.
  - Initial pose from RViz's 2D Pose Estimate (ground-truth shortcut optional).
  - Goals from RViz, then from a Simple Commander script.
- **Ch 10C What Nav2 assumes on legs** (0.5 h): one table (planar base, scan plane, no steps, circular footprint, gait IMU noise, L1 blind under 0.3 m).
- **P05 Tune Nav2** (new, 2.5 h, P2): justify settings with evidence.
  - Sweep `inflation_radius` and `cost_scaling_factor`, and view the costmap and path for each.
  - Polygon footprint vs `robot_radius` 0.33.
  - NavFn+RPP vs Smac Hybrid-A*+MPPI, 3 runs each, scored with `bag_metrics.py`.
- **P06 One behaviour tree** (new, 1.5 h, P3)
  - Reorder a recovery in `navigate_to_pose` in Groot2, then rerun the mission.
  - A waypoint patrol following the pattern of `br2_bt_patrolling`.

### F. Make it hold up, then go real (7 h)

- **P07 Test the stack** (new, 1.5 h, P2): a `launch_testing` smoke test under `colcon test`: headless sim, lidar/IMU/odom publish within N s, `map` → base resolves.
- **Ch 13 Toward the real Go2** (new, 2.5 h, P1, no hardware needed)
  - **Which Go2:** EDU (DDS over Ethernet) or AIR/PRO (WebRTC); record firmware and lidar (L1/L2).
  - **Network recipe:** static 192.168.123.x (never .161 or .18), `CYCLONEDDS_URI`, domain 0; check `ros2 topic echo /sportmodestate`.
  - **Two control levels:** never mix them. `/lowcmd` only with the robot hung up or lying down, and with low gains.
  - **Safety page:** remote in hand, Damp (1001) as e-stop, StandDown before power-off, no flips, avoidance is a separate switch, Bluetooth off (UniPwn).
  - **Topic map and bridge:** a ~30-line `cmd_vel` → SportClient Move node that clamps speeds and sends StopMove on timeout, tested on `unitree_mujoco` (domain 1, loopback) first. Move ranges and the ~1 s timeout are **S**.
  - **Bag replay:** run the ch 9 estimator on recorded `utlidar/cloud` and `utlidar/imu`. No public Go2 bag was found (**U**).
- **Ch 12** rows: `use_sim_time` in RViz, Best Effort displays, fixed frame `map` vs `odom`, Twist vs TwistStamped, Save vs Serialise.
- **P08 Capstone** (new, 3 h, P2): move an obstacle, change the start, map while navigating to 3 goals, report success over 3 runs. Objective: work alone.

### Deep dives (optional, outside the hours)

m00 and m04; a MuJoCo joint-PD lab like CS 123 Lab 1 (m07); contacts without a sensor plus a Pronto Go2 config (m11); an elevation-map costmap layer (m12; `elevation_mapping_cupy` ROS 2 support **U**); beyond CHAMP (`quadruped_ros2_control`, `rl_sar`); the IsaacLab Go2 RL task (`unitree_rl_gym` has no Go2 deploy); a Jazzy/Harmonic port onto `khaledgabr77/unitree_go2_ros2`; GrandTour scoring; executors. The 18-module roadmap is this track; no core chapter depends on it.

**Not to add:** custom Nav2 plugins, RViz panels, MoveIt, GPS, Isaac Sim, frontier exploration (the capstone maps while navigating), archived `legged_control`.

## 4. Further reading (V unless tagged)

- **Ch 1:** https://underactuated.csail.mit.edu/ (Tedrake); Raibert, *Legged Robots That Balance*, 1986: https://books.google.com/books?id=EXRiBnQ37RwC; Wensing et al., T-RO 2024: https://arxiv.org/abs/2211.11644
- **Ch 2+3:** https://gazebosim.org/docs/latest/ros_installation/; `ros2_documentation` `How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.rst`; `unitreerobotics/unitree_ros2` `.devcontainer`
- **Ch 4:** ROS `Launch/Using-Substitutions.rst`; Rico, *Concise Intro. to Robot Programming with ROS 2*, 2025: `fmrico/book_ros2`; Renard, *ROS 2 from Scratch*: `PacktPublishing/ROS-2-from-Scratch`
- **Ch 5:** ROS `Tf2/Debugging-Tf2-Problems.rst`; *Modern Robotics* ch 3: https://hades.mech.northwestern.edu/index.php/Modern_Robotics
- **Ch 6:** ROS `URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.rst`; https://articulatedrobotics.xyz/tutorials/
- **Ch 7:** `chvmp/champ`; CS 123 labs 1 and 3: https://cs123-stanford.readthedocs.io/en/latest/; `legubiao/quadruped_ros2_control`
- **P02:** `ros-teleop/twist_mux` (humble `config/`); https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/
- **P04, P06:** Rico ch 3 and 6 (`br2_fsm_bumpgo_py`, `br2_bt_patrolling`); https://docs.nav2.org/rolling/tutorials/general_tutorials/groot_tutorials/groot2/groot2/
- **Ch 8:** https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/ (sensors page); `jizhang-cmu/autonomy_stack_go2`
- **Ch 9:** Bloesch et al., RSS 2012: https://www.roboticsproceedings.org/rss08/p03.html; Pronto, Frontiers 2020: https://doi.org/10.3389/frobt.2020.00068 and `ori-drs/pronto`; VILENS: https://arxiv.org/abs/2107.07243
- **Ch 10, P05:** https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/; https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/; Marathon 2: https://arxiv.org/abs/2003.00368; `AtsushiSakai/PythonRobotics`
- **Ch 11:** GrandTour: https://arxiv.org/abs/2602.18164; Cerberus (Unitree A1/Go1 bags): https://arxiv.org/abs/2209.07654
- **Ch 12:** https://robots.ecs.soton.ac.uk/go2/troubleshooting; Articulated Robotics (gotchas)
- **P07:** ROS `Testing/Integration.rst`; `cra-ros-pkg/robot_localization` `doc/` (sign checklist)
- **Ch 13:** `unitreerobotics/unitree_ros2`; `abizovnuralem/go2_ros2_sdk`; https://robots.ecs.soton.ac.uk/go2/ros_control; UniPwn context https://arxiv.org/abs/2509.14139 (paper is on humanoids; Go2 claim from go2-real-robot.md); support.unitree.com (**U**)
