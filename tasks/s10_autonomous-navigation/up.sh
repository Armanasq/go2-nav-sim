#!/bin/bash
# scripts/up.sh on a stage-10 world:   tasks/s10_autonomous-navigation/up.sh <world> [<map>]     e.g. up.sh doorway
# Gazebo loads worlds/<world>.world, AMCL and Nav2 load worlds/<map>.yaml (default: the world's own map); a path ending in
# .world / .yaml is used as given. Same order and readiness checks as scripts/up.sh (sim -> estimation -> Nav2, Nav2 retried
# up to 3x); the passive mapper is left out. NAV=0 stops after estimation, for launch files that start their own Nav2.
HERE=$(cd "$(dirname "$0")" && pwd); W=${1:?usage: up.sh <world> [<map>]}; M=${2:-$1}
case "$W" in *.world) WF=$(realpath "$W");; *) WF="$HERE/worlds/$W.world";; esac
case "$M" in *.yaml) MF=$(realpath "$M");; *) MF="$HERE/worlds/$M.yaml";; esac
cd "$HERE/../.."; source ./env.sh
L="$HERE/logs"; mkdir -p "$L"
GZPORT=${GAZEBO_MASTER_URI##*:}
bash scripts/stop.sh; for i in $(seq 20); do ss -ltn 2>/dev/null | grep -q ":$GZPORT " || break; sleep 1; done
echo "stack: world=$W map=$M"
wait_for() { for i in $(seq "$2"); do eval "$1" >/dev/null 2>&1 && return 0; sleep 2; done; echo "TIMEOUT waiting for: $3"; return 1; }

(setsid ros2 launch go2_nav_sim sim.launch.py world:="$WF" > "$L/sim.log" 2>&1 &)
wait_for 'timeout 5 gz stats -d 1 | grep -q Factor' 75 "gazebo" || exit 1
wait_for 'timeout 10 ros2 topic echo --once /imu/data --no-daemon' 40 "robot sensors/controllers (/imu/data)" || exit 1
(setsid ros2 launch go2_nav_sim estimation.launch.py > "$L/est.log" 2>&1 &)
wait_for 'grep -q " pose=" "$L/est.log"' 30 "lidar odometry" || exit 1

[ "${NAV:-1}" = 0 ] && { echo "stack up (no nav)"; exit 0; }
ok() { [ "$(grep -c 'Managed nodes are active' "$L/nav.log")" -ge 2 ] && ! grep -q 'Aborting bringup' "$L/nav.log"; }
for try in 1 2 3; do
  (setsid ros2 launch go2_nav_sim nav.launch.py map:="$MF" mapper:=false > "$L/nav.log" 2>&1 &)
  wait_for '[ $(grep -c "Managed nodes are active" "$L/nav.log") -ge 2 ] || grep -q "Aborting bringup" "$L/nav.log"' 45 "nav2 lifecycle"
  ok && { echo "stack up"; exit 0; }
  echo "nav2 bring-up failed (attempt $try), retrying"; bash scripts/stop.sh nav
done
echo "nav2 failed to come up"; exit 1
