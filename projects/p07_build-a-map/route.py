"""The mapping route through the arena, as straight legs. No ROS in this file, so it can be unit-tested.
Waypoints are in the world frame of make_arena.py. The route starts and ends on marker A, so slam_toolbox sees the start
area again at the end and can close the loop."""
import math

A = (-4.5, -3.5)
WAYPOINTS = [            # world x, y in metres; clearance to the nearest obstacle is checked in test_mapping.py
    (4.5, -3.8),         # east along the south wall, below the pallet at (2.5, -2.5)
    (4.6, 3.8),          # north along the east wall
    (2.2, 2.1),          # into the gap between the tall brick box and the blue pillar
    (-0.8, 2.1),         # west through that gap
    (-1.0, 4.0),         # north, west of the blue pillar
    (-5.0, 4.0),         # west along the north wall
    (-5.0, 1.0),         # south along the west wall
    (-0.1, 1.0),         # east between the north brick box and the orange pillar
    (-0.1, -2.6),        # south between the tall box and the west pallet
    (-4.5, -2.6),        # west, south of the west pallet
    A,                   # back to the start: the loop closure
]
TURN_TOL, DRIVE_TOL = 0.3, 0.1   # m: a turn goal is reached at once; a drive goal ends within 10 cm


def legs(start, waypoints=WAYPOINTS):
    """Start (x, y) and waypoints -> goals [(x, y, heading), pos_tol]. Each waypoint becomes two goals: turn on the spot
    to face it, then walk to it with that heading. With the robot already facing the goal and the goal heading equal to the
    bearing, the polar law of project 4 has alpha = beta = 0 and walks a straight line, so the clearance of the straight
    segment is the clearance of the walk."""
    out, (x, y) = [], start
    for gx, gy in waypoints:
        h = math.atan2(gy - y, gx - x)
        out += [((x, y, h), TURN_TOL), ((gx, gy, h), DRIVE_TOL)]
        x, y = gx, gy
    return out


def length(start, waypoints=WAYPOINTS):
    pts = [start] + list(waypoints)
    return sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))
