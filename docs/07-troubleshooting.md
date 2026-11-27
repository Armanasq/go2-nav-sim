# 7. Troubleshooting

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
| Planner fails at once; costmap shows a blob in the middle of the room | floor points above the height threshold | `min_height: 0.30` in `config/scan.yaml` and `lidar_odom.py` |
| Planner fails; robot sits inside the inflated zone | gap too narrow for the footprint | widen the gap or use a polygon footprint |
| Planner fails; costmap has long diagonal streaks; AMCL log says `Failed to transform initial pose in time` | the initial pose was dropped in a TF timing race, so `map` to `odom` is wrong | publish `/initialpose` with a zero stamp and repeat until `/amcl_pose` converges, as `go_to_goal.py` does |
| `waitUntilNav2Active` never returns | AMCL has no initial pose in Humble's commander | `go_to_goal.py` publishes one |
| `rclpy.spin_once` loop never sees TF or odometry | it runs one callback per call, and TF arrives hundreds of times per second | drain with a loop, or give the TF listener its own thread |
| `launch`: `name 'true' is not defined` | a launch argument named `slam` leaks into Nav2's own `slam` argument | name your argument differently |
| Robot keeps walking after you stop publishing | CHAMP holds the last `cmd_vel` | publish a zero Twist |
| `scripts/stop.sh` or `pkill -f` kills your terminal | the pattern appears in your own command line | run the script on its own, never inside a command that spells the pattern |
| Gazebo cameras about 5 fps | rendering on an integrated GPU | `env.sh` enables NVIDIA PRIME offload when `nvidia-smi` exists |
