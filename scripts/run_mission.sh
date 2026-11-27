#!/bin/bash
# Fresh stack -> record bag -> drive A to B with Nav2 -> stop recording.   ./run_mission.sh bags/run1
cd "$(dirname "$0")/.."; source ./env.sh; BAG=${1:?usage: run_mission.sh <bag_dir>}
scripts/up.sh || exit 1
(setsid scripts/record_bag.sh "$BAG" > logs/bag.log 2>&1 &); sleep 6
python3 src/go2_nav_sim/scripts/go_to_goal.py | tee logs/mission.log; RC=${PIPESTATUS[0]}
sleep 4; pkill -INT -f 'ros2 bag record'; sleep 8
exit $RC
