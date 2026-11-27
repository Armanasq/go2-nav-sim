# 6. Import the Go2

The Go2 model and its gait controller are not part of this repository. `scripts/setup.sh` fetches
[anujjain-dev/unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2) at commit
`fdac259518930c8f03e79189c4d0fdd00fc47483` into `src/third_party/unitree-go2-ros2` and applies `patches/upstream.patch`. This
chapter covers what that tree contains, how this repository uses it (the patch touches only two CHAMP launch files), and what
you would provide for your own quadruped.

## What upstream provides

| Path under `src/third_party/unitree-go2-ros2` | Contents |
|---|---|
| `robots/descriptions/go2_description/xacro/` | `robot.xacro` (entry point), `leg.xacro`, `const.xacro`, `gazebo.xacro`, `materials.xacro`, and unused variants (`robot_VLP.xacro`, `velodyne.xacro`, `laser.xacro`, `transmission.xacro`) |
| `robots/descriptions/go2_description/meshes/` | Collada visuals: `trunk.dae`, `hip.dae`, `thigh.dae`, `thigh_mirror.dae`, `calf.dae`; `calf_mirror.dae`, `foot.dae` and `hokuyo.dae` are not referenced by `robot.xacro` |
| `robots/descriptions/go2_description/config/ros_control/ros_control.yaml` | controller manager and controller parameters |
| `robots/configs/go2_config/config/joints/joints.yaml` | CHAMP joint map |
| `robots/configs/go2_config/config/links/links.yaml` | CHAMP link map |
| `robots/configs/go2_config/config/gait/gait.yaml` | CHAMP gait parameters ([chapter 7](07-make-it-walk.md#gait-parameters)) |
| `champ/champ_bringup/launch/bringup.launch.py` | `robot_state_publisher`, CHAMP controller and state estimator, two EKFs |
| `champ/champ_gazebo/launch/gazebo.launch.py` | `gzserver`, spawning, controller loading |

`go2_config` also has its own `launch/gazebo.launch.py` and `bringup.launch.py` for upstream's demo. This repository does not
use them. It includes the two CHAMP launch files directly and passes the Go2 paths in.

## How the xacro is assembled

`robot.xacro` includes `const.xacro`, `materials.xacro`, `leg.xacro` and `gazebo.xacro`. It defines `base_link`, a fixed joint
named `floating_base` to `trunk` (the trunk mesh, a 0.376 × 0.094 × 0.114 m collision box and 6.921 kg), and `imu_link` at the
trunk origin. It then calls the leg macro four times:

```xml
<xacro:leg name="rf" mirror="-1" mirror_dae= "False" front_hind="1"  front_hind_dae="True"  />
<xacro:leg name="lf" mirror="1"  mirror_dae="True"   front_hind="1"  front_hind_dae="True"  />
```

`mirror` flips y offsets and inertia products for the right side, `front_hind` flips x for the hind legs, and the two `_dae`
flags choose the mesh and its orientation. `leg.xacro` creates `<name>_hip_joint` (axis x, at ±0.1934, ±0.0465 m from the trunk
centre), `<name>_upper_leg_joint` (axis y, 0.0955 m outward), `<name>_lower_leg_joint` (axis y, 0.213 m below) and a fixed
`<name>_foot_joint` another 0.213 m below. All dimensions, joint limits, masses and inertias live in `const.xacro`.

The macro also emits one `<ros2_control>` block per leg:

```xml
<ros2_control name="${name}" type="system">
    <hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware>
    <joint name="${name}_hip_joint">
        <command_interface name="effort"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>
    ...
```

`gazebo.xacro` adds the `libgazebo_ros2_control.so` plugin, pointed at
`$(find go2_description)/config/ros_control/ros_control.yaml`, an IMU on `imu_link` (100 Hz, gyro noise σ = 2e-4 rad/s, accel
noise σ = 1.7e-2 m/s², published on `/imu/data`), friction 0.6 on the feet and 0.2 on the other links, and a `p3d` plugin that
publishes `/odom/ground_truth` at 10 Hz with 1 cm noise. Three of the friction entries name `rf_hip`, `lh_hip` and `rh_hip`,
which match no link. This repository ignores `/odom/ground_truth` and adds its own noise-free `/odom_gt` at 50 Hz.

Render the model without starting anything:

```bash
source env.sh
xacro src/go2_nav_sim/urdf/go2_nav.xacro | check_urdf /dev/stdin | head -n 4
# robot name is: go2
# ---------- Successfully Parsed XML ---------------
# root Link: base_link has 4 child(ren)
#     child(1):  camera_link
xacro src/go2_nav_sim/urdf/go2_nav.xacro | grep -c 'type="revolute"'                  # 12
xacro src/go2_nav_sim/urdf/go2_nav.xacro | grep -o '<ros2_control name="[a-z]*"'      # rf, lf, rh, lh
```

## ros2_control configuration

`ros_control.yaml` sets `update_rate: 250` and declares two controllers:

- `joint_states_controller` (`joint_state_broadcaster/JointStateBroadcaster`) publishes `/joint_states` from the position and
  velocity state interfaces.
- `joint_group_effort_controller` (`joint_trajectory_controller/JointTrajectoryController`) claims the 12 effort command
  interfaces. Every joint has the gains `p: 100.0, i: 0.2, d: 1.0, i_clamp: 2.5`. CHAMP sends it position targets and the
  controller turns the position error into torque.

An identical copy of the file sits in `go2_config/config/ros_control/`. Only the one named in `gazebo.xacro` is read; the
`ros_control_file` launch argument of `champ_gazebo` is declared but never used.

## How CHAMP finds the legs

CHAMP does not parse joint names out of the URDF. It reads two maps. `joints.yaml` lists four joints per leg, in the order
`left_front`, `right_front`, `left_hind`, `right_hind`:

```yaml
joints_map:
  left_front:
    - lf_hip_joint
    - lf_upper_leg_joint
    - lf_lower_leg_joint
    - lf_foot_joint
```

Only the first three names per leg are used; they become the joint order of the trajectory CHAMP publishes. `links.yaml` gives
`base: base_link` and four links per leg (`lf_hip_link`, `lf_upper_leg_link`, `lf_lower_leg_link`, `lf_foot_link`).
`urdf_loader.h` computes the translation between consecutive links, starting from the URDF root, and builds its leg model from
those offsets. It does not read joint axes or limits. `gait.yaml` sets `knee_orientation: ">>"`: the first character is for the
front legs, the second for the hind legs, and it selects the inverse kinematics solution.

## How this repository wraps it

`src/go2_nav_sim/urdf/go2_nav.xacro` includes upstream's `robot.xacro` and this repository's `sensors.xacro`, then attaches a
lidar, an RGB-D camera, a chase camera and the ground-truth plugin to `base_link` with fixed joints
([chapter 8](08-robot-and-sensors.md)). It takes `start_x` and `start_y` so that `/odom_gt` starts at zero. The upstream
description files are not edited, so updating the pin means changing `SHA` in `setup.sh` and checking that the patch still
applies.

`src/go2_nav_sim/launch/sim.launch.py` passes the xacro path, with `start_x:=-4.5 start_y:=-3.5` appended, as `description_path`
to both CHAMP launch files, together with the three `go2_config` YAML files. Each node that needs the URDF runs `xacro` itself.

## Spawning and controller start-up

1. `robot_state_publisher` publishes the URDF on `/robot_description`.
2. `gzserver` starts with the arena world and the `gazebo_ros` init and factory plugins.
3. `spawn_entity.py -entity go2 -topic /robot_description -x -4.5 -y -3.5 -z 0.275 -Y 0.0` inserts the robot 0.275 m above the
   floor at marker A. It drops onto its feet.
4. The `gazebo_ros2_control` plugin inside the model fetches the URDF from `robot_state_publisher`, creates the four leg systems
   and starts a controller manager at `/controller_manager`.
5. Two `controller_manager` spawner nodes load and activate `joint_states_controller` and `joint_group_effort_controller`.

Upstream used `ros2 control load_controller` for step 5. That command comes from `ros2controlcli`, which is not installed here,
and without it the legs never receive a command. The patch replaces both calls with the spawner node. The launch output of one
start, shortened:

```
[spawn_entity]: Spawn status: SpawnEntity: Successfully spawned entity [go2]
[gazebo_ros2_control]: Parameter 'hold_joints' has already been declared, ...
[spawner_joint_group_effort_controller]: Configured and activated joint_group_effort_controller
[spawner_joint_states_controller]: Configured and activated joint_states_controller
```

The `hold_joints` error appears three times, once for each `<ros2_control>` block after the first, and the controllers start
normally after it. [Chapter 7](07-make-it-walk.md) checks the result.

## Adapting this to your own quadruped

1. Write a xacro whose root is `base_link` and which has three revolute joints per leg with the hip on x and the thigh and knee
   on y, plus a fixed foot link at the contact point. CHAMP's leg model (`quadruped_leg.h`, `kinematics.h`) hard-codes these
   axes. Give every link a collision shape, a mass and an inertia.
2. Add one `<ros2_control>` block per leg with `gazebo_ros2_control/GazeboSystem`, an `effort`
   command interface and `position` and `velocity` state interfaces per joint.
3. Add the `libgazebo_ros2_control.so` plugin, pointing at your controller YAML, and an IMU publishing `/imu/data`.
4. In the controller YAML, list the same 12 joint names under `joint_group_effort_controller.joints` and give each a gain entry.
   Keep the controller names: `champ_gazebo` spawns them by name, and `sim.launch.py` sends CHAMP's output to
   `joint_group_effort_controller/joint_trajectory`.
5. Write `joints.yaml` and `links.yaml` with your joint and link names under `left_front`, `right_front`, `left_hind`,
   `right_hind`, and `links_map.base` equal to your root link. The loader looks the links up by name in the URDF, so
   they must match exactly.
6. Write `gait.yaml`: set `nominal_height` below the standing height your legs can reach, `knee_orientation` to match your
   knees, and the velocity limits to what the robot can follow.
7. Point `description_path`, `joints_map_path`, `links_map_path` and `gait_config_path` in `sim.launch.py` at your files, and
   set `world_init_z` a few centimetres above the standing height so the robot drops onto its feet.

Next: [7. Make it walk](07-make-it-walk.md)
