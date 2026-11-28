#!/usr/bin/env python3
"""Top view of the Collision Monitor zones and the current /scan, in base_footprint (what RViz shows with config/arbitration.rviz).
usage: python3 projects/p11_command-arbitration/zones_snapshot.py OUT.png
Takes one message from /polygon_stop, /polygon_slowdown and /scan; prints how many scan points lie in each zone.
The monitor publishes its polygons only while velocity commands flow through it, so this sends zero twists on teleop/cmd_vel
(the robot stands) until it has all three."""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PolygonStamped, Twist
from sensor_msgs.msg import LaserScan
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle


def inside(poly, x, y):
    """Even-odd ray test: True if (x, y) lies inside the polygon [(x, y), ...]."""
    c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            c = not c
    return c


def main():
    rclpy.init()
    n = Node('p11_zones_snapshot')
    got = {}
    for name, typ, qos in (('polygon_stop', PolygonStamped, 10), ('polygon_slowdown', PolygonStamped, 10),
                           ('scan', LaserScan, qos_profile_sensor_data)):
        n.create_subscription(typ, name, lambda m, k=name: got.setdefault(k, m), qos)
    zero = n.create_publisher(Twist, 'teleop/cmd_vel', 10)
    t0 = time.time()
    while len(got) < 3 and time.time() - t0 < 20:
        zero.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.2)
    if len(got) < 3:
        sys.exit(f'no message on: {sorted({"polygon_stop", "polygon_slowdown", "scan"} - set(got))} (is the Collision Monitor active?)')
    s = got['scan']
    assert s.header.frame_id == 'base_footprint', s.header.frame_id
    pts = [(r * math.cos(s.angle_min + i * s.angle_increment), r * math.sin(s.angle_min + i * s.angle_increment))
           for i, r in enumerate(s.ranges) if s.range_min < r < s.range_max]
    fig, ax = plt.subplots(figsize=(6, 4))
    for k, col in (('polygon_slowdown', '#c27c0e'), ('polygon_stop', '#b3261e')):
        poly = [(p.x, p.y) for p in got[k].polygon.points]
        print(f'{k}: {len(poly)} corners, {sum(inside(poly, *q) for q in pts)} scan points inside')
        ax.add_patch(Polygon(poly, fill=False, ec=col, lw=2, label=k))
    ax.add_patch(Rectangle((-0.325, -0.15), 0.65, 0.30, color='#bbbbbb', label='Go2 body 0.65 x 0.30 m'))
    ax.add_patch(plt.Circle((0, 0), s.range_min, fill=False, ec='#777777', ls=':', label=f'/scan range_min {s.range_min:.2f} m'))
    ax.scatter(*zip(*pts), s=6, color='#222222', label='/scan')
    ax.set(xlim=(-1.0, 1.6), ylim=(-0.9, 0.9), aspect='equal', xlabel='x forward (m, base_footprint)', ylabel='y left (m)')
    ax.grid(color='#e5e5e5'); ax.legend(fontsize=7, loc='upper left')
    fig.tight_layout(); fig.savefig(sys.argv[1], dpi=90)
    print(sys.argv[1])
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
