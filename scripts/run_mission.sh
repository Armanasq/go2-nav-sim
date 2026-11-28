#!/bin/bash
# Fresh stack -> record bag -> drive A to B with Nav2 -> stop recording.   ./run_mission.sh bags/run1
cd "$(dirname "$0")/.."; source ./env.sh; BAG=${1:?usage: run_mission.sh <bag_dir>}
scripts/up.sh || exit 1
# setsid makes the recorder the leader of its own process group, so the stop below reaches only this recorder
# (a pkill by name would also stop the recorders of other stacks on the machine).
setsid scripts/record_bag.sh "$BAG" > logs/bag.log 2>&1 & REC=$!; sleep 6
python3 src/go2_nav_sim/scripts/go_to_goal.py | tee logs/mission.log; RC=${PIPESTATUS[0]}
sleep 4; kill -INT -- -"$REC"; wait "$REC"     # wait until the recorder has closed the bag
exit $RC
