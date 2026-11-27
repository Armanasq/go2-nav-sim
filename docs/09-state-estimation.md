# 9. State estimation

Nav2 needs an `odom` to `base_footprint` transform that is smooth and does not jump. Simulation tutorials usually take it from
the simulator. This stack estimates it from the lidar and the IMU. The odometry never reads simulator ground truth; the stack uses it once, to
initialise AMCL ([chapter 10](10-mapping-and-navigation.md#start-amcl-from-the-true-pose)), and otherwise only to score the result.

## Why not CHAMP's leg odometry

CHAMP estimates odometry from foot contacts and leg kinematics. In this Gazebo setup it was far off. On a straight walk at
0.3 m/s it reported 0.55 m after the robot had really moved 3.2 m, and drifted 0.6 m sideways while the real path was straight.
The likely cause is that nothing publishes `/foot_contacts` here ([chapter 1](01-legged-robot-foundations.md#frames)): the
Gazebo contact node that would provide them used over 200 % of a core and the patch removes it
([chapter 3](03-install-and-first-run.md#what-the-patch-changes)). Fixing leg odometry is a project of its own, so this stack
replaces it.

## Lidar odometry

`src/go2_nav_sim/scripts/lidar_odom.py` registers each scan against a local map. For a ground robot a planar (SE(2)) registration
is enough and costs a few milliseconds per scan.

1. The cloud is transformed into `base_footprint` through TF. That transform is not what it should be: CHAMP's
   `base_to_footprint` estimator receives no foot contacts here, so it publishes a constant 0.426 m height and no rotation,
   while the real trunk sits at 0.217 m and rocks by −2 to +3.6 degrees while walking. Points therefore come out 0.209 m too
   high and are not levelled. The height thresholds below are stated in this shifted frame.
2. Points between 0.30 and 1.0 m in `base_footprint` (about 0.09 to 0.79 m above the real ground) and 0.5 to 12 m away are
   kept, then voxel-downsampled to 15 cm. The lower bound removes the floor; the upper bound keeps ceilings and overhangs out of
   the scan.
3. Translation is predicted at constant velocity. The yaw increment comes from integrating the gyro between the two scans. In
   the recorded runs the integrated gyro matched ground-truth yaw to within 1 to 2 degrees over six seconds, including fast
   turns.
4. Registration is Gauss-Newton on point-to-line residuals against the local map, with Geman-McClure robust weights. It solves
   for translation only and keeps yaw at the gyro prediction; `fix_yaw:=false` gives a full SE(2) solve for a poor gyro. The map
   normals come from a PCA over eight neighbours, and neighbourhoods that are line-like (walls) count more than blobs. Translation gets a prior
   toward the prediction only along directions the scan barely constrains: the 2×2 information matrix of the translation is
   eigen-decomposed and the prior is added along eigenvectors whose eigenvalue, divided by the number of matched points, is below
   0.02 (Zhang et al., "On degeneracy of optimization-based state estimation problems", ICRA 2016). The information grows with
   the number of matches, hence the normalisation. In the arena both eigenvalues are about 90 to 100 over roughly 740 points
   (0.13 per point), so the prior is inactive; in a corridor the along-wall eigenvalue is 0 and the prior stops the pose from sliding.
5. An update that implies more than 0.6 m/s or 0.8 rad/s (plus 5 cm or 0.05 rad of slack per scan) is rejected and the
   prediction is kept, so a bad match cannot throw the pose across the room. After eight rejections in a row the local map is
   cleared and rebuilt from the next scan at the predicted pose.
6. New points are added to the map only when the robot has moved 15 cm or turned 0.1 rad, and only points within 12 m are kept.

The EKF (`robot_localization`, `config/ekf.yaml`) fuses the lidar odometry pose and velocity with the gyro yaw rate at 50 Hz and
publishes the transform. Two details matter on a busy machine:

- The TF listener runs in its own thread. A blocking transform lookup inside a subscriber callback otherwise starves the very
  thread that would deliver the transform.
- If a transform is slightly late, the node falls back to the latest one instead of dropping the scan.

## Where this sits

The estimator here is planar and written for teaching. Current systems for legged robots work in 3D and tightly couple more
sensors: FAST-LIO2 (Xu et al., arXiv:2107.06829) is a common lidar-inertial reference, and leg odometry, the velocity of the base
computed from a foot in stance, is fused with the IMU in filters such as the contact-aided invariant EKF (Hartley et al.,
arXiv:1904.09251). The leg-odometry chapter of the SLAM Handbook (2025) covers contact estimation and the filter-versus-smoother
choice. The next step for module 11 on the [roadmap](https://armanasq.github.io/go2-nav-sim/roadmap.html#m11) is that
estimator, built on the kinematics validated in module 04. For benchmarking on real legged data, the GrandTour dataset evaluates 63 open-source
estimators under one protocol.

## Unit tests

The registration lives in `registration.py`, which has no ROS dependency, so it is tested on synthetic scans of the real arena
geometry:

```bash
python3 -m pytest -q src/go2_nav_sim/test
```

The tests check that a 25 cm offset is recovered to under 2 cm from three poses, that a full SE(2) solve recovers 0.05 rad, that
20% clutter does not move the result by more than 3 cm, that a scan with no overlap is reported as a failure, that the corridor
prior holds the along-wall estimate while correcting across, and that the gyro integration is independent of callback timing.

## Check it against ground truth

```bash
python3 src/go2_nav_sim/scripts/odom_eval.py 45 &        # live, for 45 s
# then drive the sequence below with ros2 topic pub on /cmd_vel; the forward leg is
# ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}"
```

```
samples 237  path 2.40 m  ATE RMSE 0.096 m  max 0.170 m  final 0.169 m  yaw err RMSE 1.93 deg  drift 7.03 % of path
```

The run behind that output rotated in place at 0.5 rad/s, drove forward, then spun back at 1.0 rad/s (the speed of Nav2's
spin recovery). On the
three recorded navigation runs ([chapter 11](11-record-and-visualise.md)) the odometry ATE after SE(2) alignment is 1.1 to 1.7 cm over
11.6 to 13.8 m of path, and the relative error over 1 m of travel is 1.6 to 2.4 %. Read those numbers with care: the arena is small, so the 12 m local map covers the whole room and the
estimator behaves partly like localisation against a fixed map. Expect more drift in larger or featureless spaces. The method
has no loop closure, and its degeneracy test is a single threshold on the per-point information, chosen from two synthetic cases
(arena and corridor) rather than calibrated on real data.

## Failures worth knowing about

- Gyro bias is not estimated. Yaw comes from the integrated gyro and the scan matching does not correct it, so any bias turns into
  heading drift. Measurements disagree on how large it is: integrating `/imu/data` alone drifted 6.4° over 60 s standing and
  4.0° over the first 60 s of a recorded run (chapter 4's `imu_yaw` node), and one long standing test on a machine running two
  stacks saw the odometry drift 2.6 m and 22° in a few minutes; a 130 s standing test on a single stack drifted 0.27° (yaw RMSE)
  and 1.9 cm. The three missions are about 60 s long, which keeps this hidden. The fix is to estimate the bias online, for
  example from the difference between the scan-matched and the integrated yaw while the robot stands still.

- The trunk tilt never reaches TF. Found while writing chapter 7 and confirmed with `tf2_echo base_footprint base_link`
  on a running stack: a constant (0, 0, 0.426), against a ground-truth base height of 0.217 m. The IMU orientation is correct
  but unused there. The fix, still open, is to publish `base_footprint` to `base_link` from the IMU roll and pitch and a base
  height from leg kinematics (module 04). Until then, a tilt of 3.6 degrees moves a point 3 m away by about 0.19 m in height.

- IMU samples arrived late. The IMU and point-cloud callbacks shared one thread. While a registration ran, IMU samples queued up and
  were then dropped from the next scan's integration window, so yaw was under-integrated by about half during turns and the
  odometry diverged. The fix: integrate the gyro continuously into a cumulative yaw history, look it up by timestamp, and run the
  two callbacks in separate groups of a multi-threaded executor (ROS 2 guide: [Using callback groups](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html)). After this, three missions in a row succeeded.
- Other processes starved the estimator. Earlier, two repeated missions diverged (ATE 100 cm and 199 cm). At the time, three orphaned
  `lifecycle_manager` processes from earlier runs were spinning at 100% CPU each, because they ignore SIGTERM. Scans arrived late,
  the TF lookups fell back to stale data and registration was rejected.
  `scripts/stop.sh` now escalates to SIGKILL. If the odometry log shows `registration rejected`, check `top` first.
- A yaw prior made it worse. I tried tying ICP's yaw to the gyro with a stiff prior. In an earlier controlled test, whose
  output is not in the repository, the error went from under 1 cm to 2.3 m, so it is not in the code. That experiment ran
  before the late-IMU fix above, when the integrated gyro yaw was itself wrong during turns, and I have not re-tested it since.
- The rotation flipped. With a full SE(2) solve, the registration jumped to the 180-degree-flipped solution of the rectangular room
  when Nav2's spin recovery turned the robot at 1 rad/s (odometry yaw off by 175 degrees). The gyro was right all along, hence the
  translation-only solve above. The cost: gyro bias is not corrected, which is harmless in simulation and would need handling on
  hardware.
- A uniform prior biased every solution. The first version added the translation prior in all directions. A unit test showed
  the cost: with the prediction 25 cm off, the result landed 2.5 cm short of the truth, against 1 to 2 mm without the prior. The
  degeneracy-aware version above removes that bias where the scan is informative.
- Planar ICP can slide along parallel walls. The first degeneracy test used an absolute eigenvalue threshold of 10, so the same
  geometry passed or failed depending on how many points the scan had; an expert review pointed this out and the threshold is now
  per matched point (0.02). The threshold is still a hand-picked number; a real deployment would add a second source such as leg
  odometry.

Next: [10. Mapping and navigation](10-mapping-and-navigation.md)
