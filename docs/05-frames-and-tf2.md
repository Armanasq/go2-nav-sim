# 5. Frames and TF2

Every measurement on the robot is taken in some frame: the lidar measures from its own centre, the camera along its optical
axis, the IMU in its own axes. To combine them, and to say where the robot is in the room, ROS 2 keeps a tree of frames and the
transforms between them. The library is TF2. This chapter shows how to read the tree of the running stack, how lookups work in
time, and how to check a transform by hand. It also finds one transform in this stack that is wrong.

Start the stack as in [chapter 4](04-ros2-from-scratch.md#start-the-stack). The output below comes from the same session.

## What a frame is

A frame is an origin and three orthogonal axes attached to something: a link of the robot, a sensor, the floor of the room. A
transform from frame A to frame B is a rotation $R$ and a translation $t$ that give the pose of B in A. Applied to a point, it
converts B coordinates into A coordinates:

$$p_A = R\,p_B + t$$

Written as a 4×4 homogeneous matrix, a chain of transforms becomes a product of matrices:

$$T_{AB} = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}, \qquad p_A = T_{AB}\,p_B, \qquad T_{AC} = T_{AB}\,T_{BC}$$

with $p$ extended by a fourth coordinate 1. The subscripts read "B in A", and adjacent inner indices cancel.

## Axis conventions: REP-103

[REP-103](https://www.ros.org/reps/rep-0103.html) fixes units and axes for all of ROS: metres, radians, seconds, right-handed
frames, and for a body frame x forward, y left, z up. Rotations are positive anticlockwise when seen from the tip of the axis,
so a positive yaw turns the robot left. Camera optical frames are the exception: z forward along the optical axis, x right, y
down, the convention of image coordinates. This stack's `/cmd_vel` follows the same axes: the `{angular: {z: 0.5}}` command in
chapter 4 turned the robot anticlockwise.

## The standard frames: REP-105

[REP-105](https://www.ros.org/reps/rep-0105.html) names the frames of a mobile robot and the rules they follow.

| Frame | Fixed to | Rule | Published here by |
|---|---|---|---|
| `map` | the world | the robot's pose in it does not drift, but it may jump | AMCL (`map` to `odom`) |
| `odom` | the world | the robot's pose in it is continuous, but it drifts | `odom_ekf` (`odom` to `base_footprint`) |
| `base_footprint` | the robot, on the ground below the base, yaw only | | CHAMP's `base_to_footprint_ekf` (`base_footprint` to `base_link`) |
| `base_link` | the robot's base | | root of the URDF |

`base_footprint` comes from [REP-120](https://www.ros.org/reps/rep-0120.html), written for humanoids and used by most legged
and wheeled robots since.

The two world frames exist because no estimator gives both properties. Odometry integrates motion: between two scans it moves
by a small amount and never jumps, so a controller can use it to follow a path, but its error grows. A localiser such as AMCL
compares the scan with a map: its error stays bounded, but each correction can move the estimate by centimetres or more in one
step. A controller that received those steps would turn them into jerks. So the localiser does not publish the robot's pose
directly. It publishes `map` to `odom`, the correction that makes `map` to `odom` to `base_footprint` equal its estimate. The
controller works in `odom`; the planner and the goal live in `map`.

## Static and dynamic transforms

A fixed joint in the URDF never changes, so `robot_state_publisher` sends it once on `/tf_static` with transient-local
durability ([chapter 4](04-ros2-from-scratch.md#quality-of-service)); a listener that starts later still receives it. Every
other transform is streamed on `/tf` with a timestamp, and listeners keep a short history of each (10 s by default).

```bash
ros2 topic info -v /tf_static            # publisher robot_state_publisher, Durability: TRANSIENT_LOCAL
ros2 topic info -v /tf                   # publishers include odom_ekf, base_to_footprint_ekf, robot_state_publisher, amcl
```

## One parent per frame

The frames form a tree: each frame has exactly one parent, and there is exactly one path between any two frames. If two nodes
publish the same child frame, TF has no way to merge them: it keeps whichever message arrived last. With two different parents
the tree changes shape from message to message; with the same parent the transform alternates between two values. This stack has three places where that could
happen, and each is configured away:

- `sim.launch.py` starts `gt_odom.py`, which also publishes `odom` to `base_footprint`, only with `odom:=gt`. Run it that way and
  do not start `estimation.launch.py`.
- `slam_toolbox` would publish its own `map` to `odom`. `config/slam.yaml` sets `map_frame: slam_map` and
  `transform_publish_period: 0.0`, so it publishes no transform and AMCL stays the only parent of `odom`.
- CHAMP's own `footprint_to_odom` EKF is not started, through the `gt_odom` argument that the patch adds
  ([chapter 3](03-install-and-first-run.md#what-the-patch-changes)).

## Read the tree

`view_frames` listens for 5 s and writes the tree to a PDF and a Graphviz file:

```bash
ros2 run tf2_tools view_frames
# [view_frames]: Listening to tf data for 5.0 seconds...
# [view_frames]: Generating graph in frames.pdf file...
ls
# frames_2026-11-28_00.35.51.gv  frames_2026-11-28_00.35.51.pdf
```

The edges, from the `.gv` file:

| Parent to child | Average rate | Kind |
|---|---|---|
| `map` to `odom` | 10.3 Hz | dynamic (AMCL) |
| `odom` to `base_footprint` | 50.3 Hz | dynamic (EKF) |
| `base_footprint` to `base_link` | 49.8 Hz | dynamic (CHAMP) |
| `base_link` to `trunk`, `lidar_link`, `camera_link`, `chase_cam` | "10000" | static |
| `trunk` to `imu_link` and the four `*_hip_link` | static and 250 Hz | `imu_link` static; hips follow `/joint_states` |
| `*_hip_link` to `*_upper_leg_link` to `*_lower_leg_link` | 250 Hz | dynamic (`robot_state_publisher`) |
| `*_lower_leg_link` to `*_foot_link`, `camera_link` to `camera_optical`, `lidar_link` to `lidar_visual` | "10000" | static |

Static transforms show a rate of 10000 and a most recent time of 0.0: they have no time. Every edge says
`Broadcaster: default_authority`, so `view_frames` does not tell you who publishes what; `ros2 topic info -v /tf` does.

## tf2_echo

`tf2_echo A B` prints the pose of B in A, the transform that maps B coordinates into A:

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

```
At time 0.0
- Translation: [0.000, 0.000, 0.100]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
- Rotation: in RPY (radian) [0.000, -0.000, 0.000]
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]
- Matrix:
  1.000  0.000  0.000  0.000
  0.000  1.000  0.000  0.000
  0.000  0.000  1.000  0.100
  0.000  0.000  0.000  1.000
```

The lidar is 0.1 m above `base_link` and not rotated. `At time 0.0` marks a static transform. The other fixed sensors:

```bash
ros2 run tf2_ros tf2_echo base_link camera_link     # Translation: [0.280, 0.000, 0.030], no rotation
ros2 run tf2_ros tf2_echo base_link imu_link        # Translation: [0.000, 0.000, 0.000], no rotation
```

## This robot's tree, and the transform that is wrong

`odom` to `base_footprint` should carry x, y and yaw. `base_footprint` to `base_link` should carry the rest: the trunk height
above the ground, roll and pitch. That split lets TF level a sensor mounted on the trunk.

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link
```

```
At time 707.845000000
- Translation: [0.000, 0.000, 0.426]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
...
At time 708.595000000
- Translation: [0.000, 0.000, 0.426]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
```

It is stamped and published at 50 Hz, but its value is a constant. Where does it come from?

```bash
ros2 topic echo --once /base_to_footprint_pose --field pose.pose     # position z: 0.4259999990463257, identity rotation
ros2 topic echo --once /odom_gt --field pose.pose.position          # z: 0.21658939177410852
```

CHAMP's `state_estimation_node` publishes `/base_to_footprint_pose` and `base_to_footprint_ekf` turns it into the transform.
The estimator needs foot contacts, and nothing publishes `/foot_contacts` in this setup (`ros2 doctor` in chapter 4 flagged
it). 0.426 m is the height for all joint angles at zero, legs straight down: 0.213 + 0.213 m, the thigh and calf lengths
([chapter 1](01-legged-robot-foundations.md#frames) has the details). The simulator's ground truth puts `base_link` 0.217 m above
the floor. So `base_footprint` lies 0.426 − 0.217 = 0.209 m below the floor, and everything expressed in `base_footprint` or
`odom` is 0.209 m too high. Trunk roll and pitch never reach TF either.

Check it with the floor itself. The lidar's lowest beam points 15 degrees down and hits the floor in a ring about 1.2 m around
the robot. Transformed into `base_footprint`, those points should sit at z = 0 if the frame were right:

```
ring 0: 900 pts  range median 1.174 m  z in base_footprint median 0.211  mean 0.220
gt z 0.2163
```

The floor is at +0.211 m in `base_footprint`, against 0.426 − 0.2163 = 0.210 m predicted. The script that produced this
groups the points of one `/lidar/points` message by elevation angle and adds the 0.1 + 0.426 m from the two transforms above;
the next sections show how to do the same through TF.

The consequences are in [chapter 9](09-state-estimation.md#lidar-odometry): `lidar_odom.py` keeps points between 0.30 and
1.0 m in `base_footprint`, which is 0.09 to 0.79 m above the real floor. Code that needs gravity alignment takes roll and pitch
from `/imu/data` instead.

## The camera optical frame

`sensors.xacro` attaches `camera_optical` to `camera_link` with

```xml
<joint name="camera_optical_joint" type="fixed"><parent link="camera_link"/><child link="camera_optical"/><origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/></joint>
```

URDF `rpy` means: rotate about the parent's x axis by roll, then about the parent's y axis by pitch, then about the parent's z
axis by yaw. As a matrix, the last rotation is on the left:

$$R = R_z(\psi)\,R_y(\theta)\,R_x(\phi), \qquad \phi = -\tfrac{\pi}{2},\ \theta = 0,\ \psi = -\tfrac{\pi}{2}$$

With $\cos(-\frac{\pi}{2}) = 0$ and $\sin(-\frac{\pi}{2}) = -1$:

$$R_x(-\tfrac{\pi}{2}) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 & 1 \\ 0 & -1 & 0 \end{bmatrix}$$

$$R_y(0) = I$$

$$R_z(-\tfrac{\pi}{2}) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 \\ -1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

Multiply, row of the left matrix times column of the right:

$$R = \begin{bmatrix} 0 & 1 & 0 \\ -1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 & 1 \\ 0 & -1 & 0 \end{bmatrix} = \begin{bmatrix} 0 & 0 & 1 \\ -1 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix}$$

The columns of $R$ are the child's axes written in the parent frame:

$$x_{\text{optical}} = \begin{bmatrix} 0 \\ -1 \\ 0 \end{bmatrix} = -y_{\text{link}}, \qquad y_{\text{optical}} = \begin{bmatrix} 0 \\ 0 \\ -1 \end{bmatrix} = -z_{\text{link}}, \qquad z_{\text{optical}} = \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} = x_{\text{link}}$$

The optical z axis points forward along the link's x, optical x points right (minus y, since y is left) and optical y points
down. That is the optical convention of REP-103. TF agrees:

```bash
ros2 run tf2_ros tf2_echo camera_link camera_optical
```

```
At time 0.0
- Translation: [0.000, 0.000, 0.000]
- Rotation: in Quaternion (xyzw) [-0.500, 0.500, -0.500, 0.500]
- Rotation: in RPY (radian) [-1.571, 0.000, -1.571]
- Rotation: in RPY (degree) [-90.000, 0.000, -90.000]
- Matrix:
  0.000  0.000  1.000  0.000
 -1.000  0.000  0.000  0.000
 -0.000 -1.000  0.000  0.000
  0.000  0.000  0.000  1.000
```

The depth image is stamped in `camera_optical`. A pixel on the optical axis at depth $d$ is the point $(0, 0, d)$ there, and
$R\,(0,0,d)^\top = (d, 0, 0)^\top$ in `camera_link`: straight ahead, as it should be.

## How much a missing tilt costs

Because roll and pitch are not in TF, a point is placed as if the trunk were level. A point at distance $d$ along a ray that is
actually tilted by $\theta$ ends up off in height by

$$\Delta z = d \sin\theta$$

For $d = 3$ m and $\theta = 3.6°$ (the largest roll in the walk of [chapter 7](07-make-it-walk.md#how-the-trunk-moves)):
$\Delta z = 3 \times 0.0628 = 0.188$ m, about 0.19 m. The horizontal error is $d(1 - \cos\theta) = 0.006$ m, negligible. In one of the
author's recorded missions (`bags/v3`), `/odom_gt` pitch ranged from −2.95 to +1.63 degrees (3477 samples), which gives up to 0.15 m at
3 m. A 15 cm height error at 3 m is enough to move a wall point out of a height band, or to put a floor point into one.

## Lookups in time

Code uses TF through a buffer and a listener:

```python
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration

buf = Buffer()
TransformListener(buf, node, spin_thread=True)
latest = buf.lookup_transform('odom', 'base_footprint', Time())                              # the newest available
at_scan = buf.lookup_transform('base_footprint', 'lidar_link', Time.from_msg(msg.header.stamp),
                               Duration(seconds=1.0))                                       # at the scan's stamp, wait up to 1 s
```

`lookup_transform(target, source, time)` returns the transform that maps source coordinates into target coordinates, like
`tf2_echo target source`. The time argument decides which transform you get:

- `Time()` (zero) means "the latest you have", whatever its stamp. It never fails for lack of time coverage, and it may be
  stale.
- A stamp means "interpolated at this instant". For sensor data this is the right choice: the robot moved between the scan and
  now, and the latest transform would place the points where the robot is now, not where it was when it measured.

A stamped lookup fails when the buffer does not cover that time. The messages are worth reading, because they say which side
and by how much. These three came from a probe script on the running stack (the node uses `use_sim_time`):

```
tf2.ExtrapolationException: Lookup would require extrapolation into the past.  Requested time 880.096000 but the earliest data is at time 880.100000, when looking up transform from frame [lidar_link] to frame [base_footprint]
```

The script had just started. Its listener had 4 ms less history than the first scan it received needed.

```
ExtrapolationException : Lookup would require extrapolation into the future.  Requested time 935.490000 but the latest data is at time 934.979000, when looking up transform from frame [base_footprint] to frame [odom]
```

This asked for `now + 0.5 s`. No transform exists for a time that has not happened; if a stamped lookup fails into the future
by a few tens of milliseconds, the publisher is late, and a timeout (the `Duration` argument) waits for it.

```
ExtrapolationException : Lookup would require extrapolation into the past.  Requested time 904.990000 but the earliest data is at time 934.400000, when looking up transform from frame [base_footprint] to frame [odom]
```

This asked for `now − 30 s`, older than anything in the buffer. A static transform has no time and never raises these errors;
the lidar lookup in the first message failed because the chain from `lidar_link` to `base_footprint` includes the dynamic
`base_footprint` to `base_link`.

`lidar_odom.py` uses both forms: a stamped lookup with a 0.3 s timeout, and the latest transform as a fallback when the
stamped one fails on a loaded machine ([chapter 9](09-state-estimation.md#lidar-odometry)).

## Compose transforms by hand

Take one real lidar point and carry it to `odom` through the chain, then ask TF for the whole chain at once. The point is
from a `/lidar/points` message stamped 934.896 s, near the forward axis:

$$p_{\text{lidar}} = (1.2545,\ -0.0044,\ 0.0219)$$

The three transforms at that stamp, from `lookup_transform`:

$$T_{\text{base\_link}\leftarrow\text{lidar}} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0.1 \\ 0 & 0 & 0 & 1 \end{bmatrix} \qquad T_{\text{base\_footprint}\leftarrow\text{base\_link}} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0.426 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

$$T_{\text{odom}\leftarrow\text{base\_footprint}} = \begin{bmatrix} 0.9907 & -0.1362 & 0 & -0.5398 \\ 0.1362 & 0.9907 & 0 & 0.2516 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

The last one is a yaw of atan2(0.1362, 0.9907) = 7.83 degrees and a translation of (−0.540, 0.252) m. (The robot was standing
still at the time; that odometry pose is the drift measured in [chapter 4](04-ros2-from-scratch.md#check-it-against-ground-truth).)

The first two only add height: $p_{\text{base\_footprint}} = (1.2545,\ -0.0044,\ 0.0219 + 0.1 + 0.426) = (1.2545,\ -0.0044,\ 0.5479)$.
Then the rotation and translation:

$$x = 0.9907 \times 1.2545 - 0.1362 \times (-0.0044) - 0.5398 = 0.7036$$

$$y = 0.1362 \times 1.2545 + 0.9907 \times (-0.0044) + 0.2516 = 0.4181$$

$$z = 0.5479$$

TF's own answer, `lookup_transform('odom', 'lidar_link', stamp)` applied to the same point, printed by the probe:

```
p_base_footprint [ 1.2545 -0.0044  0.5479]  p_odom (composed) [0.7035 0.4181 0.5479]  p_odom (tf2 odom<-lidar_link) [0.7035 0.4181 0.5479]  diff 2.220446049250313e-16
```

The hand result differs in the fourth decimal only because the matrices above are rounded. The height shows the
`base_footprint` error again: TF puts the point 0.548 m above the ground, while the simulator has `base_link` at 0.2158 m, so the
point is really 0.2158 + 0.1 + 0.0219 = 0.338 m up. The difference is 0.210 m.

## Watch map jump and odom not

With AMCL running, publish an initial pose 0.5 m away from the true one, then the true one again, and read both transforms after
each (`/initialpose` is what RViz's "2D Pose Estimate" button publishes, and what `go_to_goal.py` sends):

```
before   map->odom ( -6.599,  -2.287, yaw  167.68 deg) at 1117.60   odom->base_footprint ( -2.354,   1.091, yaw   22.53 deg) at 1116.64
+0.5 m   map->odom ( -6.255,  -2.022, yaw  163.76 deg) at 1121.80   odom->base_footprint ( -2.422,   1.073, yaw   23.15 deg) at 1120.90
restored map->odom ( -6.826,  -2.040, yaw  164.09 deg) at 1125.70   odom->base_footprint ( -2.442,   1.097, yaw   21.96 deg) at 1124.79
```

`map` to `odom` moved by 0.43 m, then by 0.57 m, each in a single step. `odom` to `base_footprint` moved by 9 cm over the same
8 s, its slow drift, and did not react to the relocalisation. That is the REP-105 split at work.

The same output shows two more things. The odometry had drifted by 2.6 m and 22 degrees while the robot stood still for several
minutes, and `map` to `odom` had absorbed it: composing the "before" pair gives the robot at (−4.53, −3.86) and −169.8 degrees
in `map`, against (−4.60, −3.70) and −173.4 degrees from ground truth read with the robot standing at the same spot. And the `map` to `odom` stamps run about 1 s ahead of the odometry. AMCL stamps its transform
`transform_tolerance` (1.0 s in `nav2_params.yaml`) into the future, so that lookups at the current time succeed between its
updates.

## Checkpoint

- `ros2 run tf2_tools view_frames` produces a tree with `map` at the root and `odom`, `base_footprint`, `base_link` below it.
- `tf2_echo base_footprint base_link` shows the constant (0, 0, 0.426), and you can say why the floor appears at +0.21 m.
- You can derive the `camera_link` to `camera_optical` matrix from `rpy="-1.5707963 0 -1.5707963"` and match it to `tf2_echo`.
- You can read an `ExtrapolationException` and say whether the data was late, the request was in the future, or the buffer was
  too short.

## Exercises

1. A depth pixel on the optical axis reads 2.0 m. Where is that point in `base_link`? Check with `lookup_transform('base_link',
   'camera_optical', Time())`.
2. Suppose CHAMP published the true trunk height and the trunk stood at 0.216 m. What would the lowest lidar beam's floor points
   read in `base_footprint`, and what do they read in this stack?
3. In the last output above, `map` to `odom` was stamped 1117.60 while `odom` to `base_footprint` was stamped 1116.64. Which
   parameter explains the difference, and where is it set?

### Answers

1. In `camera_link` the point is $R\,(0,0,2)^\top = (2, 0, 0)$; adding the 0.28 m forward and 0.03 m up of `camera_link` gives
   (2.28, 0, 0.03) m in `base_link`. TF printed `[2.28000000e+00 5.35897930e-08 3.00000536e-02]`.
2. 0.0 m with a correct frame. In this stack 0.426 − 0.216 = 0.210 m; measured median 0.211 m over 900 points.
3. AMCL's `transform_tolerance`, 1.0 s, in `src/go2_nav_sim/config/nav2_params.yaml`. AMCL stamps `map` to `odom` that far
   ahead; the difference here is 0.96 s.

Next: [6. Import the Go2](06-import-the-go2.md)
