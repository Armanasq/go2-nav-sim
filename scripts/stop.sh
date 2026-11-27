#!/bin/bash
# Stop processes started from this workspace.   scripts/stop.sh        everything
#                                                scripts/stop.sh nav    only Nav2 / AMCL / mapper (used by up.sh to retry a failed bring-up)
# Executables are matched by full path ("/lib/<pkg>/<exe>") because `pkill -x` only sees the 15-char kernel process name and a
# plain `pkill -f name` also kills any shell whose command line merely mentions it. gzserver is launched without a path, so it
# is matched by exact name.
kill_exe() { for n in "$@"; do pkill -f -- "/(lib|bin)/([a-z0-9_]+/)?$n( |\$)" 2>/dev/null; done; }
kill9_exe() { for n in "$@"; do pkill -9 -f -- "/(lib|bin)/([a-z0-9_]+/)?$n( |\$)" 2>/dev/null; done; }   # lifecycle_manager can ignore SIGTERM and then spins at 100% CPU for hours
NAV="controller_server planner_server behavior_server bt_navigator smoother_server waypoint_follower velocity_smoother lifecycle_manager map_server map_saver_server amcl async_slam_toolbox_node"

if [ "$1" = nav ]; then
  pkill -INT -f 'ros2 launch go2_nav_sim nav.launch.py' 2>/dev/null; sleep 4; kill_exe $NAV; sleep 2; kill9_exe $NAV; exit 0
fi
pkill -x gzserver 2>/dev/null; pkill -x gzclient 2>/dev/null
kill_exe $NAV ekf_node rviz2 robot_state_publisher pointcloud_to_laserscan_node quadruped_controller_node state_estimation_node component_container_isolated
pkill -f 'ros2 launch go2_' 2>/dev/null; pkill -f 'ros2 bag record' 2>/dev/null
pkill -f 'install/go2_nav_sim/lib/go2_nav_sim/' 2>/dev/null; pkill -f 'controller_manager/spawner' 2>/dev/null
sleep 3; kill9_exe $NAV ekf_node robot_state_publisher pointcloud_to_laserscan_node quadruped_controller_node state_estimation_node; pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null; true
