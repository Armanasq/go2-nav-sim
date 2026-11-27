# 8. Robot and sensors

`src/go2_nav_sim/urdf/go2_nav.xacro` includes the upstream Go2 and adds the sensors from `sensors.xacro`. The upstream
description files are not edited. All sensors hang off `base_link` through fixed joints. In this setup `base_footprint` to
`base_link` is a constant (0, 0, 0.426 m) with no rotation, so the trunk's roll and pitch do not reach TF
([chapter 9](09-state-estimation.md)).

| Sensor | Topic | Frame | Rate | Notes |
|---|---|---|---|---|
| 3D lidar | `/lidar/points` | `lidar_link` | 10 Hz | 16 beams, ±15°, 900 samples per revolution, 0.3 to 20 m, 1 cm range noise. Mounted 0.10 m above `base_link`. Fields `x y z intensity`. |
| RGB | `/camera/image_raw` | `camera_optical` | 10 Hz | 640×360, 1.2 rad horizontal field of view |
| Depth | `/camera/depth/image_raw` | `camera_optical` | 10 Hz | 32FC1, metres, 0.3 to 8 m, 7 mm noise, `NaN` beyond range |
| IMU | `/imu/data` | `base_link` | 100 Hz | from upstream, gyro noise σ = 2e-4 rad/s |
| Ground truth | `/odom_gt` | `world` | 50 Hz | noise-free; read once by `go_to_goal.py` to initialise AMCL, otherwise used only for evaluation |
| External cameras | `/spectator/cam/*`, `/chase/cam/*` | | 10 Hz | for video only, no algorithm reads them |

`camera_optical` follows the optical convention (z forward, x right, y down) through a fixed joint with
`rpy = (-π/2, 0, -π/2)` from `camera_link`. Images are stamped in that frame, so `camera_info` and any depth-to-cloud code work
without extra rotations.

## Look at the data

```bash
ros2 topic echo --once /camera/camera_info
ros2 run rqt_image_view rqt_image_view          # pick /camera/depth/image_raw; set the colour range to 0.3 to 8
ros2 run rviz2 rviz2                            # add PointCloud2 /lidar/points, fixed frame lidar_link
```

![Sensor views](img/sensors.png)

The world has a 1 m floor grid made of visual-only lines. It gives the camera perspective and motion cues and gives you a scale
reference in every image. The arena, its obstacles and the matching occupancy map are generated from one list in
`src/go2_nav_sim/scripts/make_arena.py`, so the world and the map cannot disagree.

## Pitfall: the lidar must not see the floor as an obstacle

The lowest beams hit the floor 1.2 m ahead. Heights in `base_footprint` read 0.209 m too high, so a height threshold chosen as
if that frame were on the floor keeps floor returns, and they turn into phantom walls. [Chapter 10](10-mapping-and-navigation.md#the-height-band) shows the fix.

Next: [9. State estimation](09-state-estimation.md)
