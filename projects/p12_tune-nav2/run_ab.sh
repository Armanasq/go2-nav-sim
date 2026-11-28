#!/bin/bash
# One scored A-to-B run of a setting:   projects/p12_tune-nav2/run_ab.sh polygon ~/p12_runs/polygon_1
# Fresh stack on params/<setting>.yaml -> small bag (no images) -> go_to_goal.py -> bag closed -> score.py writes <out>.json.
# The mission log goes to <out>.log. Leaves the stack running; scripts/stop.sh stops it.
NAME=${1:?usage: run_ab.sh <setting> <out_dir>}; OUT=$(realpath -m "${2:?usage: run_ab.sh <setting> <out_dir>}")
HERE=$(cd "$(dirname "$0")" && pwd); cd "$HERE/../.."; source ./env.sh
[ -f "$HERE/params/$NAME.yaml" ] || { echo "no params/$NAME.yaml: run make_params.py first"; exit 2; }
[ -e "$OUT" ] && { echo "$OUT already exists (ros2 bag record will not overwrite it)"; exit 2; }
mkdir -p "$(dirname "$OUT")"
"$HERE/up_tuned.sh" "$HERE/params/$NAME.yaml" || exit 1
setsid ros2 bag record -o "$OUT" -s mcap /clock /tf /tf_static /odom /odom_gt /amcl_pose /plan /cmd_vel \
    /global_costmap/costmap_raw > "$OUT.bag.log" 2>&1 &
BAG=$!; sleep 6
timeout 600 python3 src/go2_nav_sim/scripts/go_to_goal.py 2>&1 | tee "$OUT.log"; RC=${PIPESTATUS[0]}
[ "$RC" = 124 ] && echo "RESULT TIMEOUT (600 s wall clock)" | tee -a "$OUT.log"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' > /dev/null   # CHAMP walks on the last command; leave it standing
sleep 3; kill -INT "$BAG"; wait "$BAG"
python3 "$HERE/score.py" "$OUT" --setting "$NAME"
exit $RC
