#!/bin/bash
# Record every topic the dashboard and the evaluation need into an MCAP bag (zstd chunks, about 10.5 MiB per simulated second):  ./record_bag.sh bags/run1
# Stop with Ctrl-C. Sim time is in the message headers, so the bag can be re-rendered or re-analysed without the simulator.
cd "$(dirname "$0")/.."; source ./env.sh
exec ros2 bag record -o "${1:?usage: record_bag.sh <bag_dir>}" -s mcap --storage-config-file src/go2_nav_sim/config/mcap_zstd.yaml \
  /clock /tf /tf_static /scan /lidar/points /imu/data /camera/image_raw /camera/depth/image_raw /camera/camera_info \
  /spectator/cam/image_raw /chase/cam/image_raw /odom /lidar_odom /odom_gt /amcl_pose /slam/map /plan /cmd_vel
