#!/bin/bash
# Bring the whole stack up in dependency order with readiness checks:  scripts/up.sh
#   sim (Gazebo + robot) -> estimation (scan, lidar odometry, EKF) -> nav (Nav2 + AMCL + passive mapper)
# Nav2's lifecycle bring-up aborts when a service call times out on a busy machine, so that step is retried (up to 3x).
cd "$(dirname "$0")/.."; source ./env.sh; mkdir -p logs
GZPORT=${GAZEBO_MASTER_URI##*:}
bash scripts/stop.sh; for i in $(seq 20); do ss -ltn 2>/dev/null | grep -q ":$GZPORT " || break; sleep 1; done     # gazebo master port free
echo "stack: ROS_DOMAIN_ID=$ROS_DOMAIN_ID GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
wait_for() { for i in $(seq "$2"); do eval "$1" >/dev/null 2>&1 && return 0; sleep 2; done; echo "TIMEOUT waiting for: $3"; return 1; }

(setsid ros2 launch go2_nav_sim sim.launch.py > logs/up_sim.log 2>&1 &)
wait_for 'timeout 5 gz stats -d 1 | grep -q Factor' 75 "gazebo" || exit 1
wait_for 'timeout 10 ros2 topic echo --once /imu/data --no-daemon' 40 "robot sensors/controllers (/imu/data)" || exit 1
(setsid ros2 launch go2_nav_sim estimation.launch.py > logs/up_est.log 2>&1 &)
wait_for 'grep -q " pose=" logs/up_est.log' 30 "lidar odometry" || exit 1

for try in 1 2 3; do
  (setsid ros2 launch go2_nav_sim nav.launch.py > logs/up_nav.log 2>&1 &)
  # ready = both lifecycle managers (localization + navigation) report active; failed = one aborted
  wait_for '[ $(grep -c "Managed nodes are active" logs/up_nav.log) -ge 2 ] || grep -q "Aborting bringup" logs/up_nav.log' 45 "nav2 lifecycle"
  if [ "$(grep -c 'Managed nodes are active' logs/up_nav.log)" -ge 2 ] && ! grep -q 'Aborting bringup' logs/up_nav.log; then echo "stack up"; exit 0; fi
  echo "nav2 bring-up failed (attempt $try), retrying"; bash scripts/stop.sh nav
done
echo "nav2 failed to come up"; exit 1
