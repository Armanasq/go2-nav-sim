#!/bin/bash
# Stop the processes of this stack.   scripts/stop.sh        everything in this ROS domain
#                                     scripts/stop.sh nav    only Nav2 / AMCL / mapper (used by up.sh to retry a failed bring-up)
# A process is stopped only if its environment has the caller's ROS_DOMAIN_ID (default 42) and its command line matches one of
# the patterns below, so a second stack in another domain is left alone. The script never touches its own ancestors, so it is
# safe to call from a shell whose command line happens to contain the same words. Escalates to SIGKILL: Nav2's
# lifecycle_manager can ignore SIGTERM and then spin at 100% CPU for hours.
DOM=${ROS_DOMAIN_ID:-42}
NAV='/lib/[a-z0-9_]+/(controller_server|planner_server|behavior_server|bt_navigator|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager|map_server|map_saver_server|amcl|async_slam_toolbox_node)( |$)|ros2 launch go2_nav_sim nav\.launch\.py'
ALL="$NAV"'|(^| |/)gz(server|client)( |$)|/lib/[a-z0-9_]+/(ekf_node|rviz2|robot_state_publisher|pointcloud_to_laserscan_node|quadruped_controller_node|state_estimation_node|component_container_isolated|spawner)( |$)|ros2 launch go2_|ros2 bag record|install/go2_nav_sim/lib/go2_nav_sim/|spawn_entity\.py'
PAT=$([ "$1" = nav ] && echo "$NAV" || echo "$ALL")

anc=" "; p=$$; while [ "${p:-1}" -gt 1 ]; do anc="$anc$p "; p=$(awk '/^PPid/{print $2}' /proc/$p/status 2>/dev/null); done
targets() {
  for d in /proc/[0-9]*; do
    pid=${d#/proc/}; [[ "$anc" == *" $pid "* ]] && continue; [ -r "$d/environ" ] || continue
    tr '\0' '\n' < "$d/environ" 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$DOM" || continue
    tr '\0' ' ' < "$d/cmdline" 2>/dev/null | grep -qE "$PAT" && echo "$pid"
  done
}
t=$(targets); [ -z "$t" ] && exit 0
kill -INT $t 2>/dev/null; sleep 3
t=$(targets); [ -n "$t" ] && kill -TERM $t 2>/dev/null && sleep 2
t=$(targets); [ -n "$t" ] && kill -KILL $t 2>/dev/null
true
