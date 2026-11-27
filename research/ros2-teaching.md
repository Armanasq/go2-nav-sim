# How ROS 2, Nav2 and simulation are taught: sources and lessons for this course

Checked 2026-11-28. "Opened" means I read the page or its source myself on that date. Star counts and last-push dates are from the GitHub API on that date.

Access notes:
- docs.ros.org (ROS 2 tutorials, and the robot_localization pages hosted there) sits behind bot protection (Anubis) and returned an "Access Denied" page. I read the same content from the source repositories instead: `ros2/ros2_documentation` on the `jazzy` branch, and `cra-ros-pkg/robot_localization` `doc/`. The live URLs below follow the site's path pattern. I did not render them.
- Nav2 docs moved to an MkDocs layout under `docs.nav2.org/rolling/...`. Old Sphinx URLs such as `docs.nav2.org/setup_guides/index.html` and `docs.nav2.org/tuning/index.html` return 404. The slam_toolbox README still links the old `docs.nav2.org/tutorials/docs/navigation2_with_slam.html`, which is now dead. Any link in our chapters to the old Nav2 paths needs updating.

## Annotated sources

### Official documentation

**1. ROS 2 Tutorials (Jazzy).** https://docs.ros.org/en/jazzy/Tutorials.html. Source: https://github.com/ros2/ros2_documentation (1,076 stars, pushed 2026-11-27). Opened via source; live page blocked.
The tutorial tree goes in this order:
- Beginner CLI Tools: environment, turtlesim, nodes, topics, services, parameters, actions, rqt_console, launching multiple nodes, bag record/play.
- Beginner Client Libraries: workspace, colcon, package, pub/sub and service/client in C++ and Python, custom interfaces, parameters in a class, ros2doctor, pluginlib.
- Intermediate: rosdep, actions, composition, launch, tf2, testing, URDF, RViz.

Every page opens with the same header: **Goal / Tutorial level / Time / Prerequisites / Background / Steps**. The steps are numbered sub-steps, each with a command and its expected output.

The material most relevant to us:
- **URDF** (in order): visual model from scratch ("One Shape", "Multiple Shapes", "Origins", "Material Girl", "Finishing the Model"), then movable model, then physical and collision properties, then xacro, then robot_state_publisher in C++ and Python. Each stage grows the same R2D2 file one change at a time.
- **tf2**: quaternion fundamentals, static broadcaster, broadcaster, listener, adding a frame, tf2 and time, time travel, and "Debugging tf2 problems". That last one ships a deliberately broken listener and walks through `tf2_echo`, `tf2_monitor` and `view_frames` to find the faults.
- **Launch**: creating launch files, substitutions (`DeclareLaunchArgument` / `LaunchConfiguration`), event handlers, and "launch for large projects", which covers YAML parameter files and per-world launch files.
- **Testing**: CLI, C++ (gtest), Python (pytest), and integration tests with `launch_testing` (Integration.rst, 20 min). That page launches nodes plus unittest-based "active" and "post-shutdown" tests, registers them in CMake so `colcon test` runs them, and gives each test a unique ROS domain ID.
- **RViz User Guide**: displays, configurations, views, coordinate frames, tools, time.
- **How-To guides**: "Setup ROS 2 with VSCode and Docker" (a `.devcontainer/` folder with a devcontainer.json and Dockerfile; community-contributed) and "Run 2 nodes in single or separate docker containers".

**2. Nav2 First-Time Robot Setup Guide.** https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/. Source: https://github.com/ros-navigation/docs.nav2.org (193 stars, pushed 2026-11-24). Code: https://github.com/ros-navigation/navigation2_tutorials (377 stars, pushed 2026-11-16). Opened (live title checked, source read).
Fixed order:
1. Transformations
2. URDF
3. SDF
4. Odometry
5. Smoothing odometry with robot_localization
6. Sensors (LaserScan, PointCloud2, Range, Image; Gazebo plugins; updating the ros_gz bridge config)
7. Mapping and localization
8. Footprint (configure it, then visualise it in RViz)
9. Navigation plugins

One toy robot, `sam_bot`, is built across every page. Each page ends with "Build, Run and Verification" and an RViz check. The guide states it now targets Gazebo Harmonic or newer with ROS 2 Jazzy or newer.

**3. Nav2 Tuning Guide.** https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/. Opened.
Sections: inflation potential fields, footprint vs radius, rotate-in-place behaviour, planner selection, controller selection, Smac obstacle-heuristic caching, Costmap2D plugins, symmetric yaw tolerance, launch options, and RMW/composition/QoS performance.
Points that matter for us:
- The maintainers say many configs "are really missing the point of the inflation layer". They recommend raising cost scale and radius so the whole map has a smooth potential.
- Use `robot_radius` only for truly circular robots, or with holonomic planners (NavFn, Theta*, Smac 2D), which ignore the SE2 footprint anyway.
- The supported-robot table lists **Legged** under Smac Hybrid-A* (planner) and under MPPI, Regulated Pure Pursuit (RPP) and the VP controller (controllers).
- DWB is listed for differential and omnidirectional robots only.

**4. Nav2 tutorials, Getting Started, BT docs.** https://docs.nav2.org/rolling/. Opened (titles of the pages below verified live).
- Quickstart: install, run the example, navigate.
- Dev Container guide: build the image, start the container, terminals, lifecycle, rebuilding.
- "Nav2 Behavior Trees", with a detailed BT walkthrough and Groot / Groot2 tutorials.
- General tutorials. Relevant ones:
  - Navigating while Mapping (SLAM): robot interfaces, then Nav2, then SLAM, then working with SLAM.
  - Using VIO to augment odometry.
  - Collision Monitor.
  - Keepout and speed filters.
  - Rotation Shim controller.
  - Adding a smoother to a BT.
  - "Docker for Development: Zero to Hero": key commands, ROS images, a dev image, visualisation from Docker, deployment.
  - Ground terrain segmentation with 3D lidar (ground consistency layer).
  - Get backtrace and profiling.
  - Simple Commander API.

**5. robot_localization docs.** Live: https://docs.ros.org/en/jazzy/p/robot_localization/ (blocked). Source: https://github.com/cra-ros-pkg/robot_localization `doc/` (1,992 stars, pushed 2026-11-02). Opened via source.
Pages: state estimation nodes, preparing sensor data, configuring, navsat_transform, integrating GPS, lifecycle support. "Preparing Your Data" is effectively a beginner checklist:
- Read REP-103 and REP-105.
- IMU data must be ENU.
- When a sensor gives both position and velocity, fuse the velocity. When it gives both orientation and angular velocity, fuse the orientation.
- With two orientation sources, fuse only the more trustworthy one, or use `_differential`.
- Do not inflate covariances to "ignore" a variable. It hurts the filter.
- Check signs: turn counter-clockwise and yaw must increase; drive forward and x must increase.
- Turn off the driver's own odom->base_link TF if the EKF publishes it.

**6. slam_toolbox.** https://github.com/SteveMacenski/slam_toolbox (2,736 stars, pushed 2026-11-02). Opened README.
- It describes itself as the currently supported ROS 2 SLAM library.
- Modes: synchronous and asynchronous mapping, lifelong mapping, pose-graph localization (optionally without a prior map, as "lidar odometry"), multi-robot.
- It has an RViz plugin for saving or serialising maps and for graph manipulation.
- The configuration is documented as solver, toolbox and matcher params.

**7. ros2_control docs and demos.** Docs: https://control.ros.org/jazzy/index.html (opened). Demos: https://github.com/ros-controls/ros2_control_demos (877 stars, pushed 2026-11-18). Gazebo plugin: https://github.com/ros-controls/gz_ros2_control (262 stars, pushed 2026-11-20).
- Docs sections: Getting Started, ros2_control, ros2_controllers, Demos, Simulator Integrations.
- The demos are a numbered ladder of 18 examples: RRBot, then DiffBot, multiple interfaces, sensors, transmissions, Gazebo simulation (9), controller chaining, and finally #18 "Open Duck Mini with MuJoCo simulation and ONNX policy". That last one is the only legged example.
- ros2_controllers (master) has no legged or quadruped gait controller. It has diff/mecanum/omni/steering, joint trajectory, PID, and IMU/pose broadcasters.

**8. Gazebo Harmonic tutorials.** https://gazebosim.org/docs/harmonic/tutorials/. Opened.
- Sequence: building your own robot (SDF), moving the robot, SDF worlds, sensors, actors, GUI, manipulating models, Fuel models, keyboard shortcuts, spawn URDF.
- ROS 2 how-tos: integration overview, launching Gazebo from ROS 2, interacting via ROS 2, simulation interfaces, spawning a model, interoperability, the integration template, web visualisation.
- "Moving the robot" grows the previous tutorial's SDF. It adds the DiffDrive plugin, drives the robot with `gz topic -t /cmd_vel`, then maps keys to Twist commands with KeyPublisher and TriggeredPublisher.
- Related repositories:
  - https://github.com/gazebosim/ros_gz (553 stars, pushed 2026-11-27).
  - https://github.com/gazebosim/ros_gz_project_template (242 stars, pushed 2026-09-16). It splits a project into `_description`, `_gazebo`, `_application` and `_bringup` packages. Branch `main` targets Harmonic and later; branch `fortress` targets Fortress.

**9. teleop_twist_joy.** https://github.com/ros2/teleop_twist_joy (86 stars, pushed 2025-07-16). Opened repo metadata only. Its usage is taught in source 10.

### Independent series

**10. Articulated Robotics (Josh Newans).** https://articulatedrobotics.xyz/tutorials/. Code: https://github.com/joshnewans/articubot_one (507 stars, last push 2024-10-08). Opened the pages listed here.
The sections are "Getting ready for ROS" (Linux, install, packages, TF, URDF, Gazebo, networking), "Build a Mobile Robot", "Docker for Robotics" (overview, 101, Dockerfile, devices, dev containers) and "Coordinate Transforms". The written pages are Jazzy with modern Gazebo (`ros_gz`); the repository is older. Every page lists its gotchas explicitly.
- **URDF Design**: built up one part at a time:
  1. empty `base_link` at the rotation centre
  2. chassis on a fixed joint
  3. wheels on continuous joints, rolled ±π/2
  4. caster
  5. collision copied from visual
  6. inertia macros
  7. RViz check with `joint_state_publisher_gui`, RobotModel and TF displays, toggling visual vs collision

  Gotchas: rebuild once for new files even with `--symlink-install`; restart robot_state_publisher after URDF edits; the root link should not carry inertia.
- **Gazebo**: spawn from `/robot_description` with `ros_gz_sim create` at z=0.1, bridge config in `gz_bridge.yaml`, DiffDrive and JointStatePublisher plugins, TwistStamped on `/cmd_vel`. Gotchas:
  - `use_sim_time` on every node
  - if a world has any plugin, the defaults are not loaded, so list Physics, UserCommands, SceneBroadcaster, Sensors and Contact explicitly
  - `mu1` in URDF becomes `mu` in SDF
- **ros2_control**: controller manager, hardware interfaces, command vs state interfaces, `<ros2_control>` tag, `gz_ros2_control` plugin, `my_controllers.yaml`, spawners, `ros2 control list_hardware_interfaces`.
- **Teleop**: keyboard, then `joy_node` (`joy_enumerate_devices`), then `teleop_twist_joy` (enable/deadman and turbo buttons), with `twist_mux` arbitrating keyboard, joystick and Nav2. Test in sim first, then on the real robot propped up, then on the ground. Gotchas: `stamped:=true`, remap each source to its own topic, prefer USB over Bluetooth pads, use Best Effort QoS when RViz shows nothing.
- **SLAM**: `mapper_params_online_async.yaml`, mapping vs localization mode, "Save Map" (pgm/yaml) vs "Serialise Map" (posegraph), files saved in RViz's working directory.
- **Nav2**: copy the default `nav2_params.yaml` and edit it; `enable_stamped_cmd_vel` on Jazzy; RViz fixed frame `map`; Transient Local durability for a prerecorded map; AMCL jumps can add spurious obstacles.

**11. Automatic Addison (Addison Sears-Collins), ROS 2 Jazzy series.** Example opened: https://automaticaddison.com/building-a-map-of-the-environment-using-slam-ros-2-jazzy/ (posted 2024-12-02, updated 2025-06-24). The site index refused connection on two of three attempts.
Mobile-manipulator series on a simulated Yahboom ROSMASTER X3 (mecanum) in Gazebo with ros2_control and Nav2. The SLAM page:
- adds `config/` and `maps/` folders
- writes a Twist-to-TwistStamped relay node
- adds a `slam` launch argument
- drives the robot with `rqt_robot_steering`
- saves the map with `map_saver_cli`
- cleans the `.pgm` in GIMP (back up first)

Search results (S) also list Jazzy posts on installing Nav2, autonomous navigation and the Simple Commander API. I did not open those.

**12. The Construct (Robot Ignite Academy).** https://www.theconstruct.ai/robotigniteacademy_learnros/ros-courses-library/. Opened catalog.
ROS 2 courses include: Basics in 5 Days (C++/Python), Navigation, Advanced Navigation, Intermediate ROS 2, TF ROS 2, ROS 2 Control Framework, GTest for ROS 2, Behavior Trees for ROS 2, Introduction to Gazebo Sim with ROS 2, and Build Your First ROS2 Based Robot. The catalog lists no legged or quadruped course. The catalog page does not describe the course format (browser simulation, exams); that is unverified here.

**13. Robotics Back-End (Edouard Renard).** https://roboticsbackend.com/. **Unverified**: the connection was refused on every attempt, so nothing from it is used here.

### Books (with code repositories)

**14. F. Martín Rico, *A Concise Introduction to Robot Programming with ROS 2*, 2nd ed., Chapman & Hall/CRC, 2025.** https://www.routledge.com/A-Concise-Introduction-to-Robot-Programming-with-ROS-2/Rico/p/book/9781032851488 (opened). Code: https://github.com/fmrico/book_ros2 (844 stars, pushed 2026-03-26; branches for Humble, Jazzy, Kilted and Rolling; slides in PDF, Keynote and PPT).
- Chapters: Introduction; First Steps; First Behavior: avoiding obstacles with FSMs; the TF subsystem; reactive behaviours; behaviour trees; Deep ROS 2; contributing.
- Packages: `br2_fsm_bumpgo`, `br2_vff_avoidance`, `br2_bt_bumpgo`, `br2_bt_patrolling` (BT driving Nav2 waypoints), `br2_tf2_detector`, `br2_navigation`.
- The book teaches obstacle avoidance twice, first as an FSM and then as a BT, on the same task. It is the only source here that does that.

**15. L. Joseph, J. Cacace, *Mastering ROS 2 for Robotics Programming*, 4th ed., Packt.** Code: https://github.com/PacktPublishing/Mastering-ROS-2-for-Robotics-Programming (242 stars, pushed 2026-11-25, repo created 2024-07-25). The Packt product page returned 403, so the publication year is unverified; the README says it covers ROS 2 Jazzy and "the new Gazebo Sim".
Chapters from the README: Introduction; programming; concepts; 3D modelling; simulation; ros2_control; BehaviorTree.CPP; Nav2; MoveIt 2; perception; aerial; DIY mobile robot; testing and CI/CD; LLMs; deep RL; visualisation and simulation plugins.

### Legged precedent

**16. CHAMP.** https://github.com/chvmp/champ (2,343 stars). The `ros2` branch was last committed 2023-02-24; its README lists Humble. It already provides the sequence quadruped in Gazebo, then teleop, then SLAM (slam_toolbox), then Nav2, then `map_saver_cli`, then autonomous navigation. Our course builds on it. Which Gazebo version the `ros2` branch uses was not checked.

## What a course like ours should take from this

Comparison basis: `docs/00`–`12`, `config/` and `src/go2_nav_sim/config/` in this repo. The course uses Humble with Gazebo Classic 11, NavFn plus RPP, and `robot_radius: 0.33`. It has no joystick, twist_mux, launch_testing, BT, Groot or Docker material.

1. **Give every chapter the ROS docs header.** Goal / Level / Time / Prerequisites, then numbered steps. Each step ends with a check the learner can see: an expected `ros2 topic hz` rate, an RViz screenshot, or a `tf2_echo` line. Close the chapter with "Summary / Next".
2. **Grow one artifact one change at a time.** Gazebo grows one SDF; Articulated Robotics grows `robot_core.xacro`; Nav2 grows `sam_bot`. For the Go2 the URDF already exists, so make chapter 6 a reading-and-modifying exercise:
   - load it with `joint_state_publisher_gui`
   - move one leg joint
   - toggle visual vs collision in RViz
   - add one sensor link and confirm it with `view_frames`
3. **Teach tf2 debugging with a deliberately broken setup** (ROS docs, Debugging tf2). In chapter 5, ship a launch file with one wrong static transform and one missing frame. The learner finds both with `tf2_echo`, `tf2_monitor` and `view_frames`.
4. **Add a teleop chapter before autonomy.** Steps:
   1. keyboard (`teleop_twist_keyboard`)
   2. gamepad (`joy_node`, `joy_enumerate_devices`, `teleop_twist_joy` with the deadman button)
   3. `twist_mux`, so the joystick always overrides Nav2

   Follow the order sim, then robot suspended or held, then floor at reduced speed. For the real Go2 the mux and deadman are the safety layer.
5. **Make launch arguments and YAML parameters a hands-on task.** There is no `DeclareLaunchArgument` anywhere in `docs/`. Exercise:
   1. expose `use_sim_time`, `world` and `params_file` as launch arguments
   2. override one Nav2 value from the command line
   3. confirm it with `ros2 param get` and `ros2 param dump`
6. **Justify the Nav2 plugin choice against the tuning guide.** The guide lists Legged under Smac Hybrid-A* and under MPPI/RPP. We use NavFn plus `robot_radius`, which the guide accepts only with a holonomic planner. Say that explicitly. Add one comparison exercise: NavFn plus RPP vs Smac Hybrid-A* plus MPPI with a rectangular Go2 footprint, measured on our metrics (time, path length, success over N runs).
7. **Make one costmap-tuning task with a visible result.** Change `inflation_radius` and `cost_scaling_factor` following the "smooth potential field" advice, view `/global_costmap/costmap` in RViz, and compare the paths. Also show the footprint in RViz (Nav2 footprint page).
8. **Run robot_localization's sign and frame checklist as a test.** Rotate CCW and assert yaw increases; walk forward and assert x increases; fuse only one orientation source; make sure no node besides the EKF publishes `odom->base_link`. This fits the existing pytest style for chapter 9.
9. **Teach slam_toolbox's two kinds of saved map.** Cover Save Map (pgm/yaml for AMCL) vs Serialise Map (posegraph for slam_toolbox localization), with `map_saver_cli` as the scripted path. Also teach the Transient Local durability needed to see a loaded map in RViz.
10. **Add a single BT exercise, not a BT chapter.** Open Nav2's default `navigate_to_pose` BT in Groot2. Change one thing (recovery order, or add a smoother per the "Adding a Smoother to a BT" tutorial) and rerun the mission. Rico's `br2_bt_patrolling` is the reference pattern for a waypoint-patrol BT.
11. **Add one `launch_testing` smoke test.** Launch the sim headless, assert that `/lidar/points`, `/imu/data` and `/odom` publish within N seconds and that TF `map->base_link` resolves, and register it so `colcon test` runs it. This would be the course's first ROS-level test; the existing unit tests run without ROS.
12. **Provide a dev container.** Humble with Gazebo Classic 11 needs Ubuntu 22.04, and Classic is end-of-life. A `.devcontainer/` pinned to that stack lets learners on 24.04 run the course. The ROS docs VSCode+Docker how-to, Nav2's "Docker for Development" and the Articulated Robotics Docker series all show the pattern, including GUI forwarding.
13. **Collect the recurring gotchas into troubleshooting.** Every source above repeats them:
    - `use_sim_time` on every node, including RViz
    - Twist vs TwistStamped (Jazzy Nav2 needs `enable_stamped_cmd_vel`)
    - Best Effort QoS for sensor displays in RViz
    - fixed frame `map` vs `odom`
    - rebuild after adding files under `--symlink-install`
    - world SDF plugins switch off Gazebo's default plugins (Harmonic)
14. **Plan the Jazzy/Harmonic port on the official layout.** Nav2's setup guide now targets Harmonic with Jazzy. Use `ros_gz_project_template`'s split (`_description`, `_gazebo`, `_application`, `_bringup`) and a `gz_bridge.yaml`, and keep chapter numbering stable so only chapters 3, 6 and 8 change.

Skipped as over-scope for a beginner Go2 course:
- writing custom RViz panels or Gazebo systems
- pluginlib / custom Nav2 plugins (Nav2 "Plugin Tutorials")
- GPS / navsat
- MoveIt
- DDS tuning beyond the Cyclone choice the course already makes
