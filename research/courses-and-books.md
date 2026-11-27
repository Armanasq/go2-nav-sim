# Courses and textbooks for a Go2 / ROS 2 beginner course

Compiled 2026-11-28. Angle: university courses and textbooks on legged robots and robot software, and what a course that takes a beginner from zero to autonomous Go2 navigation in ROS 2 should take from them.

Verification tags:
- **V**: I opened the page or repository and confirmed what the entry says.
- **S**: search-result snippet only, because the publisher page was paywalled or returned 403.
- **U**: not confirmed.
- **(inference)**: my own reading, not a claim made by the source.

Star counts and last-push dates come from `gh api repos/...` on 2026-11-28.

---

## 1. Annotated sources

### A. University courses

**1. MIT 6.832 Underactuated Robotics (Russ Tedrake). Online text: https://underactuated.csail.mit.edu/ (V). Companion repo: https://github.com/RussTedrake/underactuated (V; 1048 stars, last push 2026-11-26).**
- The online text is "Algorithms for Walking, Running, Swimming, Flying, and Manipulation", © 2024, Spring 2024 edition.
- Teaching order:
  - Model systems: pendulum; acrobot, cart-pole and quadrotor; Ch 4 simple models of walking and running; Ch 5 highly-articulated legged robots.
  - Then DP, LQR, Lyapunov, trajectory optimisation, motion planning, contact.
  - Then estimation and learning: system ID, state estimation, policy search, imitation learning.
- Ch 4 (V) covers:
  - limit cycles and Poincaré maps
  - rimless wheel, compass gait and kneed walker
  - SLIP with the apex-to-apex map and SLIP control
  - the MIT Leg Lab hoppers, including "running on four legs as though they were one"
  - Its only exercise is a one-dimensional hopper based on Raibert (1984).
- Ch 5 (V) covers:
  - a floating-base "spacecraft" thought experiment
  - CoP and ZMP, the ZMP planner, and CoM-to-whole-body planning
  - centroidal dynamics and spatial momentum, then whole-body control
  - Exercises: mixed-integer footstep planning, a ZMP free-body analysis, and footstep planning with GCS.
  - Quadrupeds get only a passing mention, and the LittleDog section is empty.
- Course format (Spring 2024 page, V):
  - Python notebooks graded on Gradescope by an autograder (35%), a final project (35%) and a midterm. Lectures are on YouTube.
  - Prerequisites are linear algebra and ODEs.
- Relevance: the reference for "why legged robots are hard" (underactuation, a floating base, contact). It contains no ROS.

**2. ETH Zurich Robot Dynamics, Robotic Systems Lab. Course page: https://rsl.ethz.ch/education-students/lectures/robotdynamics.html (V, HS 2024 page).**
- Topic order on the page:
  1. kinematics, dynamics and rotations of multi-body systems
  2. robot arms and legged systems, including kinematic constraints and interaction forces
  3. fixed-wing aircraft
  4. rotorcraft
- Exercises are done in MATLAB on the student's own computer. There are two midterms.
- The page does not name the lecturers (U).
- The RSL lecture notes (HS 2017) are only visible through third-party mirrors (S). Their TOC includes floating-base dynamics and floating-base inverse dynamics, and they stress that a floating base "can typically not directly be controlled through actuators but only through external forces such as contact".
- Relevance: the clearest source for the floating-base and contact-constraint view a legged course needs. It contains no ROS.

**3. ETH Zurich "Programming for Robotics – ROS", Robotic Systems Lab. Course page: https://rsl.ethz.ch/education-students/lectures/ros.html (V, 2026 edition).**
- The 2026 edition is now ROS 2. Lecturers: L. Werner, L. Terenzi, P. Eyschen, M. Hutter. Five half-day sessions in February 2026.
- Objectives, in the order listed:
  1. ROS 2 architecture: nodes, topics, messages, services, parameters, actions
  2. CLI tools
  3. packages, launch files and best practices
  4. rclcpp
  5. Gazebo, URDF and SDF
  6. RViz and rqt
  7. internals: TF, time and bags
- Format: a guided tutorial, then 3 graded exercises (20% each) of increasing difficulty "on an autonomous robot", then a project-based final exam (40%).
- The robot used in 2026 is not stated (U). The course material ZIP linked from the page did not download (it returned a 3.6 kB non-zip file), so the exercises are unverified (U). Docker setup instructions are on Moodle.
- Related repo: https://github.com/leggedrobotics/ros_best_practices (V; 1639 stars, last push 2022-01-22). It is a wiki of ROS conventions. Its README says the ROS 2 version is "in the making", with a `foxy` branch.

**4. Stanford CS 123 "A Hands-On Introduction to Building AI-Enabled Robots". Course site: https://cs123-stanford.readthedocs.io/en/latest/ (V, Fall 2026).**
- Instructors: Jie Tan and Stuart Bowers. Co-instructor: Karen Liu.
- Robot: Pupper, a quadruped students build themselves, with a Raspberry Pi 5 running ROS 2. The page does not name the ROS 2 distro (U).
- Weekly lecture and lab pairs:
  1. ROS intro and PD control
  2. forward kinematics
  3. IK and heuristic walking
  4. RL for locomotion
  5. onboard computer vision ("Follow Me Pupper")
  6. foundation models and HRI
  7. an open final project
- Lab 1 (V), on a single knee joint:
  - Inspect `/joint_states` with `ros2 topic`.
  - Write bang-bang, then P, then PD control (Kp 2.0, Kd 0.3).
  - Sweep the gains (Kp 0.5–5, Kd 0.1–1), inject a sensing delay, and track a sine wave.
- Lab 3 (V):
  - IK by gradient descent, checked offline first.
  - Triangle swing-trajectory interpolation.
  - Trot keyframes on all 12 joints through `ros2 launch`, ending in a timed 10-foot race.
- Format: groups of 3. Deliverables are a shared document with answers, code, screenshots and videos. The robot stays on a stand unless told otherwise.
- Relevance: the closest existing course to ours (a quadruped, ROS 2, starting from zero). It stops at vision and does not cover mapping or navigation.

**5. NYU Tandon, Reinforcement Learning and Optimal Control for Autonomous Systems I (Ludovic Righetti), Go2 project. Project page: https://machines-in-motion.github.io/RL_class_go2_project (V; year not stated).**
- Students train a Unitree Go2 walking policy with PPO in Isaac Lab, starting from a weak velocity-tracking baseline.
- Reward terms to add: posture, foot clearance, foot slip, smooth actions, contact regularisation, collisions.
- Then add domain randomisation over speed, disturbances and ground.
- Benchmark metrics: velocity-tracking error, orientation error, slip count, episode length, energy proxy.
- No ROS 2. No hardware step is listed.
- Relevance: a template for an optional "where the walking controller comes from" module on Go2.

**6. Modern Robotics specialization on Coursera (Northwestern, Kevin Lynch). Specialization page: https://www.coursera.org/specializations/modernrobotics (V).**
- Six courses:
  1. Foundations of Robot Motion (24 h)
  2. Robot Kinematics (19 h)
  3. Robot Dynamics (22 h)
  4. Motion Planning and Control (32 h)
  5. Manipulation and Wheeled Mobile Robots (35 h)
  6. Capstone: mobile manipulation (21 h)
- Assessments are coding projects in Python, MATLAB or Mathematica on the book's library. Software is checked in the V-REP/CoppeliaSim simulator. The capstone has a KUKA youBot doing pick-and-place.
- Relevance: a model for "short video plus quiz plus a simulator-checked coding project" units. It contains no ROS and no legs.

**7. Cyrill Stachniss, Robot Mapping (recorded in Freiburg, 2013) and Mobile Sensing and Robotics I/II (Bonn).**
- The listing on https://www.ipb.uni-bonn.de/teaching/ is V: it shows the courses and that they have YouTube recordings.
- The Robot Mapping course page linked there returned 404, so its lecture list is U.
- Relevance: free lecture video that follows *Probabilistic Robotics* for SLAM and state estimation. It is useful as optional viewing for the mapping module.

### B. Textbooks (theory)

**8. Lynch and Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, 2017. Book page: https://hades.mech.northwestern.edu/index.php/Modern_Robotics (V).**
- Chapters:
  1. Preview
  2. Configuration Space
  3. Rigid-Body Motions
  4. Forward Kinematics
  5. Velocity Kinematics and Statics
  6. Inverse Kinematics
  7. Closed Chains
  8. Dynamics of Open Chains
  9. Trajectory Generation
  10. Motion Planning
  11. Robot Control
  12. Grasping and Manipulation
  13. Wheeled Mobile Robots
- A free preprint PDF ("version 2", Dec 2019) is on the book page, and video lectures are linked there.
- Code library: https://github.com/NxRLab/ModernRobotics (V; 3090 stars, last push 2026-02-18). It is in Python, MATLAB and Mathematica, and the repo describes it as "easy to read and educational", not optimised.
- For our course:
  - Ch 3 (rigid-body motions and frames) underlies TF.
  - Ch 4–6 cover leg FK and IK.
  - Ch 13 (omnidirectional and nonholonomic bases, odometry) is the closest textbook model for commanding a Go2 by body velocity (inference).

**9. Siciliano, Sciavicco, Villani and Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009.**
- Metadata and description are V via https://ch.mathworks.com/support/books/book49007.html: ISBN 978-1-84628-641-4, "over 150 end-of-chapter problems", later chapters on dynamics, motion and interaction control, mobile robots and motion planning.
- Chapter order is S (Springer redirects to login): Introduction, Kinematics, Differential Kinematics and Statics, Trajectory Planning, Actuators and Sensors, Control Architecture, Dynamics, Motion Control, Force Control, Visual Servoing, Mobile Robots, Motion Planning, appendices.
- Relevance: a reference for the instructor, not for beginners. Its Mobile Robots and Motion Planning chapters are a second source for the navigation theory.

**10. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008.**
- The citation is V on the author's publication list (https://users.cecs.anu.edu.au/~roy/publist.html) and in Open Library, dated 2007-11-30.
- Companion MATLAB package spatial_v2: https://royfeatherstone.org/spatial/ (V). It "implements most of the algorithms" in the book.
- The 11-chapter TOC was not opened (U). From snippets: spatial vector algebra, RNEA, CRBA, ABA and floating-base dynamics.
- Relevance: background for anyone who later uses Pinocchio, MuJoCo or Drake. A beginner Go2 course should only name it.

**11. Thrun, Burgard and Fox, *Probabilistic Robotics*, MIT Press, 2005.**
- The publisher and date are V in Open Library. The full brief contents are V from a scanned TOC (https://www.obalkyknih.cz/file/toc/156816/pdf).
- Contents:
  - I Basics: 1 Introduction, 2 Recursive State Estimation, 3 Gaussian Filters, 4 Nonparametric Filters, 5 Robot Motion, 6 Robot Perception.
  - II Localization: 7 Markov and Gaussian localization, 8 Grid and Monte Carlo localization.
  - III Mapping: 9 Occupancy Grid Mapping, 10 SLAM, 11 GraphSLAM, 12 SEIF, 13 FastSLAM.
  - IV Planning and Control: 14 MDPs, 15 POMDPs, 16 Approximate POMDP, 17 Exploration.
- Exercises are at the end of each chapter (V, e.g. section "2.8 Exercises" in the TOC).
- Relevance: the theory spine for localization and mapping. Ch 8 (MCL) is what Nav2 AMCL implements, Ch 9 is the costmap and occupancy-map idea, Ch 10–11 cover graph SLAM as in slam_toolbox, and Ch 17 is frontier exploration (inference for the mapping to ROS packages).
- Companion code: https://github.com/AtsushiSakai/PythonRobotics (V; 30988 stars, last push 2026-11-23). It is short Python scripts plus an online "textbook": EKF/PF/histogram localization, grid maps, ICP, FastSLAM, A*/D*/DWA/RRT, MPC path tracking, and one "bipedal planner with inverted pendulum".

**12. Raibert, *Legged Robots That Balance*, MIT Press, 1986. Google Books record: https://books.google.com/books?id=EXRiBnQ37RwC (V).**
- Contents:
  1. Introduction
  2. Hopping on One Leg in the Plane
  3. Hopping in Three Dimensions
  4. Biped and Quadruped Running
  5. Symmetry in Running
  6. Alternatives for Locomotion Control
  7. Tabular Control of Running
  8. Research on Animals and Vehicles
- Relevance: the source of the three-part controller (hop height, body attitude, forward speed through foot placement) and of the idea that a trot runs "as though one leg". Tedrake Ch 4 re-teaches it with notebooks. One lecture's worth for a beginner course.

**13. Kajita, Hirukawa, Harada and Yokoi, *Introduction to Humanoid Robotics*, Springer Tracts in Advanced Robotics 101, 2014.**
- Title, publisher and date are V in Open Library (ISBN 9783642545351). It is a translation of a 2005 Ohmsha original (S).
- The chapter list is S from a library record: Kinematics; ZMP and Dynamics; Biped Walking; Generation of Whole-Body Motion Patterns; Dynamic Simulation. Springer reports 6 chapters, so one title is missing (U).
- Relevance: the standard derivation of ZMP, the linear inverted pendulum and preview control. It is aimed at bipeds. For a quadruped course it supplies the "support polygon and CoM" intuition only.

### C. ROS 2 books (2023–2026)

**14. Francisco Martín Rico, *A Concise Introduction to Robot Programming with ROS 2*, 2nd ed., Chapman & Hall/CRC (Routledge), 2025. Publisher page: https://www.routledge.com/A-Concise-Introduction-to-Robot-Programming-with-ROS-2/Rico/p/book/9781032851488 (V).**
- Chapters:
  1. Introduction
  2. First Steps with ROS 2
  3. First Behavior: Avoiding Obstacles with Finite State Machines
  4. The TF Subsystem
  5. Reactive Behaviors
  6. Programming Robot Behaviors with Behavior Trees
  7. Deep ROS 2
  8. Contributing to ROS 2 Software
- 258 pages.
- Code: https://github.com/fmrico/book_ros2 (V; 844 stars, last push 2026-03-26).
  - Branches: rolling, kilted-devel, jazzy-devel, humble-devel and foxy-devel, plus Pixi/RoboStack setup.
  - Packages: `br2_basics`, `br2_fsm_bumpgo_{cpp,py}`, `br2_tf2_detector`, `br2_vff_avoidance` (virtual-force-field avoidance), `br2_bt_bumpgo`, `br2_bt_patrolling`, `br2_navigation` (launch, maps, params), `br2_tracking`, `br2_tiago`.
  - Slides in PDF, Keynote and PPT are linked from the README.
- Simulated robot: Tiago, inferred from `br2_tiago`.
- Relevance: the best match for our task order. A laser-based obstacle-avoidance FSM is the first behaviour, then TF, then reactive control, then behaviour trees, then Nav2 patrolling.

**15. Edouard Renard, *ROS 2 from Scratch*, Packt, Nov 2024. Code: https://github.com/PacktPublishing/ROS-2-from-Scratch (V; 291 stars, created 2024-02, last push 2026-02-14).**
- README (V): ROS 2 Jazzy and Gazebo Harmonic on Ubuntu 24.04, Python with optional C++.
  - Topics: nodes, topics, services, actions, custom interfaces, parameters, launch files, TF, URDF and RViz, then spawning and controlling the robot in Gazebo.
- Part 3 chapter titles are S (Packt pages returned 403): 10 Discovering TFs with RViz, 11 Creating a URDF for a Robot, 12 Publishing TFs and Packaging the URDF, 13 Simulating a Robot in Gazebo. The repo has a `ch13/my_robot_description` folder, which is consistent with that.
- Relevance: the gentlest path from "never used ROS" to "my robot model moves in Gazebo". It covers no navigation.

**16. Lentin Joseph and Jonathan Cacace, *Mastering ROS 2 for Robotics Programming*, 4th ed., Packt. Code: https://github.com/PacktPublishing/Mastering-ROS-2-for-Robotics-Programming (V; 242 stars, created 2024-07, last push 2025-09-16).**
- README chapters (V):
  1. Intro
  2. Programming
  3. Concepts
  4. 3D modelling
  5. Simulation (Gazebo Sim)
  6. ros2_control
  7. BehaviorTree.CPP
  8. Nav2
  9. MoveIt 2
  10. Perception
  11. Aerial
  12. DIY mobile robot
  13. Testing and CI/CD
  14. LLM agents
  15. Deep RL (Isaac Lab/Sim)
  16. Visualization and simulation plugins
- Targets ROS 2 Jazzy. The exact publication year is U.
- Relevance: a reference for the Nav2, ros2_control, BT and testing chapters. It is too wide to serve as the course spine.

---

## 2. What a course like ours should take from this

These are concrete items. Where an item is my inference it says so. "Must not skip" means a topic that every relevant source above treats as foundational, or that a Go2 learner would hit as a failure without it.

1. **Copy the ETH ROS course and Renard order for the first third.** Teach ROS 2 graph, then CLI, then packages and launch, then URDF/Gazebo/RViz, then TF, time and bags.
   - Both sources put TF, `use_sim_time` and bags inside the beginner block, not at the end.
   - Must not skip: TF, sim time, recording and replaying a bag. Every later Go2 failure (an empty costmap, AMCL not converging, a "frame does not exist" error) traces back to these (inference).
2. **The first behaviour should be laser obstacle avoidance with a finite-state machine, as in Rico Ch 3 (`br2_fsm_bumpgo`).**
   - On Go2 it would read the LiDAR `LaserScan` or point cloud and publish `cmd_vel`, in Python with no Nav2.
   - It gives "move and avoid obstacles" in week 1–2, before any theory.
3. **Then reactive avoidance (VFF, Rico Ch 5), then behaviour trees (Rico Ch 6, Mastering ROS 2 ch 7) before Nav2.** Nav2 is driven by BehaviorTree.CPP, so students who wrote one small BT can read Nav2's default navigate BT instead of treating it as magic.
4. **Teach the Go2 base as a velocity-commanded body with limits, using Modern Robotics Ch 13 (wheeled/omni base kinematics and odometry) as the model.** State explicitly that the locomotion controller underneath is a black box with speed, step-height and terrain limits. (inference: the Go2 high-level interface takes body vx, vy and yaw-rate, which matches the omni-base model; confirm against the unitree_ros2 message definitions.)
5. **Spend one lecture on "why a legged base is different", not a dynamics course.** Use these three ideas from Tedrake Ch 4–5, the ETH RD notes and Raibert:
   1. The floating base is moved only through contact forces.
   2. Support polygon, CoP and ZMP give static and dynamic balance intuition.
   3. Raibert's foot-placement rule for speed, and trot as a "virtual one-legged" gait.

   Must not skip: the floating-base and contact idea. It explains why leg odometry drifts, why the IMU matters, and why stairs or slopes fail.

   Skip for beginners: Featherstone algorithms, ZMP preview control derivations, centroidal MPC, whole-body control. Cite them as further reading.
6. **One joint-level lab in simulation, modelled on CS 123 Lab 1.**
   - Command one Go2 joint with PD (kp, kd) in simulation.
   - Sweep the gains, add an artificial delay, and track a sine wave. Deliverable: plot or video.
   - It builds intuition for stiffness and damping and for why low-level mode is dangerous on hardware.
   - Keep it sim-only. CS 123 keeps the robot on a stand. A real Go2 in low-level mode needs the same rule, written as a lab safety requirement (inference).
7. **The mapping and navigation module should follow *Probabilistic Robotics* order, mapped to the ROS 2 packages that implement each idea:**
   1. Bayes filter and noise (Ch 2–4)
   2. motion and sensor models (Ch 5–6)
   3. MCL, i.e. AMCL (Ch 8)
   4. occupancy grids and costmaps (Ch 9)
   5. SLAM and graph SLAM, i.e. slam_toolbox or a lidar SLAM (Ch 10–11)
   6. exploration and frontiers (Ch 17)

   Must not skip: the occupancy grid, MCL and a pose graph. Use PythonRobotics scripts as 20-line demonstrations before the ROS package.
8. **Exercise format.** Each module gets:
   - a runnable simulation check: an autograded or scripted pass/fail, as in MIT 6.832 on Gradescope and Coursera MR on CoppeliaSim
   - a short evidence deliverable: video, screenshot or bag, as in CS 123 lab documents
   - a project-style final on the robot, as in the ETH ROS final (40%)

   For us: a final mission where Go2 in simulation maps an unknown area, then navigates to goals while avoiding a moved obstacle (inference).
9. **Learned locomotion is now standard course content (CS 123 Lab 4, the NYU Go2 Isaac Lab project). Offer it only as an optional module.** Use the NYU reward list (velocity tracking, posture, foot clearance, slip, smoothness, collisions) and its metrics. It is not needed for the core goal of moving, mapping and navigating, and it needs a GPU (inference).
10. **The gap in this material that our course fills (inference from the TOCs above).**
    - No ROS 2 book here uses a legged robot: Rico uses Tiago, Renard uses a custom wheeled URDF, and Joseph and Cacace use wheeled and aerial robots.
    - No legged course here reaches mapping and navigation: CS 123 stops at vision, and MIT and ETH have no ROS.
    - Topics nobody covers end to end for a quadruped in ROS 2:
      - the vendor bridge (unitree_ros2 or the SDK, and how it maps to `cmd_vel`, `odom` and `tf`)
      - legged odometry and its drift
      - sensor mounting frames in the URDF
      - Nav2 tuning for an omni-capable but step-limited base
      - 2.5D or elevation maps for terrain the robot can step over
      - the hand-off from sim to the real robot

    These are the "missing" items to write ourselves, kept short.
11. **Do not over-build.** The theory books (Siciliano, Featherstone, Kajita, Tedrake Ch 5) are instructor references. Link to one chapter each as optional reading and do not turn them into lectures (inference, consistent with the user's "avoid over-detailing").

### Not verified, and why
- The full TOCs of Siciliano, Featherstone and Kajita: Springer pages redirect to a login.
- The full TOC of Renard: Packt returns 403.
- The 2026 ETH ROS exercises: the ZIP did not download.
- The Bonn Robot Mapping lecture list: the course page returned 404.
- The ETH Robot Dynamics lecturer names: not on the course page.
- The ROS 2 distro used by CS 123.
- The publication year of *Mastering ROS 2* 4th ed.
