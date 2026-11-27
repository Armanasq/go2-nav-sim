# 4. State estimation

Nav2 needs an `odom` to `base_footprint` transform that is smooth and does not jump. Simulation tutorials usually take it from
the simulator. This stack estimates it from the lidar and the IMU, and uses simulator ground truth only to score the result.

## Why not CHAMP's leg odometry

CHAMP estimates odometry from foot contacts and leg kinematics. In this Gazebo setup it was far off. On a straight walk at
0.3 m/s it reported 0.55 m after the robot had really moved 3.2 m, and drifted 0.6 m sideways while the real path was straight.
The upstream launch file sets `publish_foot_contacts: false`, which is the likely cause; the foot-contact node it would need
costs a lot of CPU (see the patch notes). Fixing leg odometry is a project of its own, so this stack replaces it.

## Lidar odometry

`src/go2_nav_sim/scripts/lidar_odom.py` registers each scan against a local map. For a ground robot a planar (SE(2)) registration
is enough and costs a few milliseconds per scan.

1. **Project.** The cloud is transformed into `base_footprint` through TF. Because the TF chain carries the IMU roll and pitch, the
   result is gravity-aligned even while the trunk rocks during the gait.
2. **Clean.** Keep points between 0.30 and 1.0 m above the ground and 0.5 to 12 m away, then voxel-downsample to 15 cm. The lower
   bound removes the floor; the upper bound keeps ceilings and overhangs out of the scan.
3. **Predict.** Constant velocity for translation. The yaw increment comes from integrating the gyro between the two scans. In
   the recorded runs the integrated gyro matched ground-truth yaw to within 1 to 2 degrees over six seconds, including fast
   turns.
4. **Register.** Gauss-Newton on point-to-line residuals, solving for translation only (yaw stays at the gyro prediction; set
   `fix_yaw:=false` for a full SE(2) solve when the gyro is poor), against the local map, with Geman-McClure robust weights. The map normals
   come from a PCA over eight neighbours, and neighbourhoods that are line-like (walls) count more than blobs. A weak Tikhonov
   term pulls the translation toward the prediction. It is negligible where the scan constrains the pose and takes over along a
   wall, where the scan alone does not.
5. **Gate and recover.** An update that implies more than 0.6 m/s or 0.8 rad/s is rejected and the prediction is kept, so a bad
   match cannot throw the pose across the room. After eight rejections in a row the local map is rebuilt from the current scan.
6. **Map.** New points are added only when the robot has moved 15 cm or turned 0.1 rad, and only points within 12 m are kept.

The EKF (`robot_localization`, `config/ekf.yaml`) fuses the lidar odometry pose and velocity with the gyro yaw rate at 50 Hz and
publishes the transform. Two details matter on a busy machine:

- The TF listener runs in its own thread. A blocking transform lookup inside a subscriber callback otherwise starves the very
  thread that would deliver the transform.
- If a transform is slightly late, the node falls back to the latest one instead of dropping the scan.

## Check it against ground truth

```bash
python3 src/go2_nav_sim/scripts/odom_eval.py 45 &        # live, for 45 s
# drive with: ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}"
```

```
samples 237  path 2.40 m  ATE RMSE 0.096 m  max 0.170 m  final 0.169 m  yaw err RMSE 1.93 deg  drift 7.03 % of path
```

That test rotates in place at 0.5 rad/s, drives forward, then spins back at 1.0 rad/s (the speed of Nav2's spin recovery). On the
three recorded navigation runs ([chapter 6](06-record-and-visualise.md)) the odometry ATE is 2.3 to 5.1 cm over 11.7 to 15.1 m of
path, 0.2 to 1.5 % drift. Read those numbers with care: the arena is small, so the 12 m local map covers the whole room and the
estimator behaves partly like localisation against a fixed map. Expect more drift in larger or featureless spaces, where this
method has no loop closure and no check for degenerate geometry such as a long corridor.

## Failures worth knowing about

- **Late IMU samples.** The IMU and point-cloud callbacks shared one thread. While a registration ran, IMU samples queued up and
  were then dropped from the next scan's integration window, so yaw was under-integrated by about half during turns and the
  odometry diverged. The fix: integrate the gyro continuously into a cumulative yaw history, look it up by timestamp, and run the
  two callbacks in separate groups of a multi-threaded executor. After this, three missions in a row succeeded.
- **Starved by other processes.** Earlier, two repeated missions diverged (ATE 100 cm and 199 cm). At the time, three orphaned
  `lifecycle_manager` processes from earlier runs were spinning at 100% CPU each, because they ignore SIGTERM. Scans arrived late,
  the TF lookups fell back to stale data and registration was rejected. After killing them, the same controlled test gave 0.8 cm.
  `scripts/stop.sh` now escalates to SIGKILL. If the odometry log shows `registration rejected`, check `top` first.
- **A yaw prior made it worse.** I tried tying ICP's yaw to the gyro with a stiff prior. The error went from under 1 cm to
  2.3 m in the same test, so it is not in the code. I did not find out why the gyro-based yaw disagrees with ICP.
- **Rotation flips.** With a full SE(2) solve, the registration jumped to the 180-degree-flipped solution of the rectangular room
  when Nav2's spin recovery turned the robot at 1 rad/s (odometry yaw off by 175 degrees). The gyro was right all along, hence the
  translation-only solve above. The cost: gyro bias is not corrected, which is harmless in simulation and would need handling on
  hardware.
- **Geometry.** Planar ICP can slide along parallel walls. Even on a clean machine, one run in three diverged (ATE 100 cm) while
  driving along a wall, which is what the weak translation prior above is for. A long corridor would be worse. A real deployment
  would add an explicit degeneracy check or a second source such as leg odometry.

Next: [5. Mapping and navigation](05-mapping-and-navigation.md)
