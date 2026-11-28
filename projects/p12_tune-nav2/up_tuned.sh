#!/bin/bash
# scripts/up.sh with your own Nav2 params file:   projects/p12_tune-nav2/up_tuned.sh projects/p12_tune-nav2/params/polygon.yaml
# Same order and readiness checks as scripts/up.sh (sim -> estimation -> Nav2 + AMCL, Nav2 retried up to 3x), but Nav2 is started
# from nav2_bringup with the file you give, and there is no passive mapper. Logs go to $P12_LOGS (default ~/p12_runs/logs).
PARAMS=$(realpath "${1:?usage: up_tuned.sh <nav2_params.yaml>}") || exit 2
cd "$(dirname "$0")/../.."; source ./env.sh
L=${P12_LOGS:-$HOME/p12_runs/logs}; mkdir -p "$L"
GZPORT=${GAZEBO_MASTER_URI##*:}
bash scripts/stop.sh; for i in $(seq 20); do ss -ltn 2>/dev/null | grep -q ":$GZPORT " || break; sleep 1; done
echo "stack: ROS_DOMAIN_ID=$ROS_DOMAIN_ID GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI params=$PARAMS"
wait_for() { for i in $(seq "$2"); do eval "$1" >/dev/null 2>&1 && return 0; sleep 2; done; echo "TIMEOUT waiting for: $3"; return 1; }

(setsid ros2 launch go2_nav_sim sim.launch.py > "$L/sim.log" 2>&1 &)
wait_for 'timeout 5 gz stats -d 1 | grep -q Factor' 75 "gazebo" || exit 1
wait_for 'timeout 10 ros2 topic echo --once /imu/data --no-daemon' 40 "robot sensors/controllers (/imu/data)" || exit 1
(setsid ros2 launch go2_nav_sim estimation.launch.py > "$L/est.log" 2>&1 &)
wait_for 'grep -q " pose=" "$L/est.log"' 30 "lidar odometry" || exit 1

MAP=$(ros2 pkg prefix go2_nav_sim)/share/go2_nav_sim/maps/arena.yaml
ok() { [ "$(grep -c 'Managed nodes are active' "$L/nav.log")" -ge 2 ] && ! grep -q 'Aborting bringup' "$L/nav.log"; }
for try in 1 2 3; do
  (setsid ros2 launch nav2_bringup bringup_launch.py map:="$MAP" params_file:="$PARAMS" use_sim_time:=true autostart:=true \
      use_composition:=False > "$L/nav.log" 2>&1 &)
  wait_for '[ $(grep -c "Managed nodes are active" "$L/nav.log") -ge 2 ] || grep -q "Aborting bringup" "$L/nav.log"' 45 "nav2 lifecycle"
  ok && { echo "stack up"; exit 0; }
  echo "nav2 bring-up failed (attempt $try), retrying"; bash scripts/stop.sh nav
done
echo "nav2 failed to come up"; exit 1
