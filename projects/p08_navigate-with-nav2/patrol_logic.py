"""The parts of the patrol that need no ROS: the waypoints, where to drop the surprise box, and how a run is scored.
Tested by test_patrol_logic.py."""
import math

import numpy as np

A = (-4.5, -3.5)     # marker A, the start (make_arena.py)


def arrival_yaws(points, start=A):
    """(x, y, yaw) per point, yaw = heading of the straight line from the previous point: the robot can stop facing the
    way it came instead of turning on the spot at the goal (see "If it goes wrong" in README.md)."""
    prev, out = start, []
    for x, y in points:
        out.append((x, y, math.atan2(y - prev[1], x - prev[0]))); prev = (x, y)
    return out


# Four waypoints (map frame, metres): east along the south wall, north up the east side, back down through the gap between
# the brick box and the pallet, home to A. Two earlier routes through the north half lost AMCL ("If it goes wrong").
WAYPOINTS = arrival_yaws([(4.0, -3.8), (4.0, 1.5), (0.0, -3.5), A])
BOX = 0.8            # side of the surprise box (m); its height is also 0.8 m, inside the /scan band of 0.09-0.69 m above the floor
SPAWN_LEG = 1        # drop it on the leg to waypoint 1 (index), the long straight up the east side
SPAWN_AFTER = 1.5    # ...once the robot has walked this far on that leg (m)
SPAWN_AHEAD = 2.0    # ...on the current plan this far ahead of the robot (m)
ROBOT_RADIUS = 0.33  # robot_radius in config/nav2_params.yaml


def path_length(xy):
    """Length of a polyline given as an (N, 2) array."""
    xy = np.asarray(xy, float)
    return float(np.hypot(*np.diff(xy, axis=0).T).sum()) if len(xy) > 1 else 0.0


def point_ahead(plan, robot, dist):
    """The point of `plan` (N, 2) that lies `dist` metres further along it than the plan point nearest to `robot`.
    Clamped to the last point."""
    plan = np.asarray(plan, float)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(plan, axis=0).T))])
    i0 = int(np.argmin(np.hypot(*(plan - robot).T)))
    i = min(int(np.searchsorted(s, s[i0] + dist)), len(plan) - 1)
    return float(plan[i, 0]), float(plan[i, 1])


def resample(xy, step=0.02):
    """The polyline xy (N, 2) with points at most `step` apart, so a segment cannot jump over a thin obstacle."""
    xy = np.asarray(xy, float)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    q = np.linspace(0, s[-1], max(int(s[-1] / step) + 2, 2))
    return np.c_[np.interp(q, s, xy[:, 0]), np.interp(q, s, xy[:, 1])]


def box_clearance(xy, centre, side=BOX):
    """Smallest distance from the polyline xy (N, 2) to an axis-aligned square box; 0 if it enters the box."""
    d = np.abs(resample(xy) - centre) - side / 2                    # ponytail: axis-aligned only, the box is spawned with yaw 0
    return float(np.hypot(*np.maximum(d, 0).T).min())


def first_clear_plan(plans, t_spawn, centre, radius=ROBOT_RADIUS):
    """Index of the first plan stamped after t_spawn that keeps the robot circle off the box, or None.
    plans: list of (t, xy) in time order."""
    for i, (t, xy) in enumerate(plans):
        if t > t_spawn and len(xy) and box_clearance(xy, centre) > radius:
            return i
    return None


def box_sdf(name, side=BOX):
    """SDF of a static orange cube for gazebo_ros /spawn_entity. Its origin is the cube centre."""
    g = f'<geometry><box><size>{side} {side} {side}</size></box></geometry>'
    return (f"<sdf version='1.6'><model name='{name}'><static>true</static><link name='l'>"
            f"<collision name='c'>{g}</collision><visual name='v'>{g}"
            "<material><ambient>0.9 0.4 0.1 1</ambient><diffuse>0.9 0.4 0.1 1</diffuse></material></visual></link></model></sdf>")


def leg_times(t_start, arrivals, t_end):
    """Durations of the legs from the times the follower moved on (arrivals, one per waypoint but the last) and the end."""
    t = [t_start, *arrivals, t_end]
    return [round(b - a, 1) for a, b in zip(t, t[1:])]
