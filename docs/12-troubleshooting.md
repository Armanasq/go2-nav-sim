# 12. Troubleshooting

Each entry is a failure that happened while building this.

| Symptom | Cause | Fix |
|---|---|---|
| Gazebo shows sensors, but no `/scan`, `/imu/data` or camera topics, and there is no error | `libgazebo_ros_*` plugins not installed | install `ros-humble-gazebo-plugins`, or let `setup.sh` unpack it into `overlay/` |
| Robot stands limp, nothing moves on `/cmd_vel` | upstream launch calls `ros2 control`, which needs `ros2controlcli` | the patch switches to the `controller_manager` spawner |
| `colcon build` fails with `No module named 'em'` | a virtualenv is first on `PATH` | `source env.sh`, which strips it; rebuild with `--cmake-clean-cache` |
| `lifecycle_manager ... Failed to bring up all requested nodes`, `change_state ... client will not receive response` | Fast DDS lost a service reply on a busy machine | use Cyclone DDS (`env.sh` does); `scripts/up.sh` also retries the Nav2 step |
| `gzserver` exits with code 255 right after a restart | the previous `gzserver` still holds port 11345 | `scripts/stop.sh`, wait until `ss -ltn \| grep 11345` is empty |
| Camera topics at 5 to 6 Hz in a Python script | the script, not the sim: Python deserialises a 0.9 MB message slowly | measure with `ros2 bag record` and `ros2 bag info`, which counts messages in C++ |
| Planner: "goal is off the global costmap" | the map covers only what has been seen | navigate on a complete map, or add exploration |
| Planner fails at once; costmap shows a blob in the middle of the room | `base_footprint` sits 0.209 m below the real ground (constant TF), so a 0.12 m threshold is below the floor and every floor return is an obstacle | `min_height: 0.30` in `config/scan.yaml` and `lidar_odom.py` (about 0.09 m real) |
| `tf2_echo base_footprint base_link` shows a constant (0, 0, 0.426) while the robot walks | CHAMP's `base_to_footprint` estimator gets no foot contacts: nothing publishes `/foot_contacts` in Gazebo | open: publish the transform from IMU roll and pitch plus a kinematic base height ([chapter 9](09-state-estimation.md#failures-worth-knowing-about)) |
| Planner fails; robot sits inside the inflated zone | the plan went through a gap that the footprint fits but that is high inflation cost all the way across | widen the gap, or tune inflation and use a polygon footprint |
| Planner fails; costmap has long diagonal streaks; AMCL log says `Failed to transform initial pose in time` | the initial pose was dropped in a TF timing race, so `map` to `odom` is wrong | publish `/initialpose` with a zero stamp and repeat until `/amcl_pose` converges, as `go_to_goal.py` does |
| `waitUntilNav2Active` never returns | AMCL has no initial pose in Humble's commander | `go_to_goal.py` publishes one |
| `rclpy.spin_once` loop never sees TF or odometry | it runs one callback per call, and TF arrives hundreds of times per second | drain with a loop, or give the TF listener its own thread |
| `launch`: `name 'true' is not defined` | a launch argument named `slam` leaks into Nav2's own `slam` argument | name your argument differently |
| Robot keeps walking after you stop publishing | CHAMP holds the last `cmd_vel` | publish a zero Twist |
| `pkill -f` kills your own terminal (exit code 144) | the pattern appears in your shell's command line, and `pkill -f` matches it too | `scripts/stop.sh` now skips its own ancestors; with plain `pkill -f`, never type the pattern in the same command |
| Stopping one stack kills another one on the same machine | `pkill` by name ignores ROS domains | `scripts/stop.sh` only stops processes whose environment has the caller's `ROS_DOMAIN_ID`; give each stack its own domain and `GAZEBO_MASTER_URI` port |
| Gazebo cameras about 5 fps | rendering on an integrated GPU | `env.sh` enables NVIDIA PRIME offload when `nvidia-smi` exists |
