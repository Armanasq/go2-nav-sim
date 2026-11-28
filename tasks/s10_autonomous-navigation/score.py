"""ROS-free scoring of a stage-10 run from the true track. Tested in tests/test_score.py.
gt: [(t, x, y, yaw)] in the map frame; amcl: [(t, x, y)]; world: an entry of worlds/make_worlds.WORLDS."""
import math, pathlib, sys
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / 'worlds'))
P02 = HERE.parents[1] / 'projects/p02_teleop-and-safety'
import make_worlds as mw


def path_length(xy):
    """Length of a track. Points closer than 5 cm to the last kept one are skipped, so the side-to-side sway of the trot
    does not add length."""
    kept = [xy[0]]
    for p in xy[1:]:
        if math.hypot(p[0] - kept[-1][0], p[1] - kept[-1][1]) >= 0.05:
            kept.append(p)
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(kept, kept[1:])))


def leg(gt, amcl, t0, t1, goal, world):
    g = [r for r in gt if t0 <= r[0] <= t1 + 2.0] or gt[-1:]
    xy = [(r[1], r[2]) for r in g]
    end = gt[-1] if gt[-1][0] <= t1 + 2.0 else g[-1]
    out = dict(final_error_m=round(math.hypot(end[1] - goal[0], end[2] - goal[1]), 3),
               path_m=round(path_length(xy), 2),
               straight_m=round(math.hypot(goal[0] - xy[0][0], goal[1] - xy[0][1]), 2),
               min_clearance_m=round(min(mw.clearance(world, x, y) for x, y in xy), 3),
               max_displacement_m=round(max(math.hypot(x - xy[0][0], y - xy[0][1]) for x, y in xy), 3))
    T = np.array([r[0] for r in gt])
    e = [math.hypot(x - gt[int(np.argmin(abs(T - t)))][1], y - gt[int(np.argmin(abs(T - t)))][2])
         for t, x, y in amcl if t0 <= t <= t1]
    out['amcl_rmse_m'] = round(float(np.sqrt(np.mean(np.square(e)))), 3) if e else None
    out['amcl_max_m'] = round(float(max(e)), 3) if e else None
    return out


def box_dist(px, py, box, size):
    """Distance from points to the surface of a yawed box (0 inside)."""
    c, s = math.cos(box['yaw']), math.sin(box['yaw'])
    dx, dy = np.asarray(px) - box['x'], np.asarray(py) - box['y']
    u, v = c * dx + s * dy, -s * dx + c * dy
    return np.hypot(np.maximum(abs(u) - size[0] / 2, 0), np.maximum(abs(v) - size[1] / 2, 0))


def replan(plans, gt, box, size, t_end, radius=0.33):
    """Did the plan at the spawn cross the box, and how long until a plan kept robot_radius away from it."""
    before = [p for t, p in plans if t <= box['t'] and len(p)]
    after = [(t, p) for t, p in plans if box['t'] < t <= t_end and len(p)]
    clear = [t for t, p in after if box_dist(p[:, 0], p[:, 1], box, size).min() >= radius]
    track = [r for r in gt if box['t'] <= r[0] <= t_end]
    return dict(plan_blocked_at_spawn=bool(before and box_dist(before[-1][:, 0], before[-1][:, 1], box, size).min() < radius),
                replan_s=round(clear[0] - box['t'], 2) if clear else None,
                min_box_clearance_m=round(float(box_dist([r[1] for r in track], [r[2] for r in track], box, size).min()), 3)
                if track else None)


def stop(gt, t_cancel):
    """Time from the cancel to standstill (p02's rule) and distance walked from the cancel to the end of the track."""
    sys.path.insert(0, str(P02))
    from measure_stop import stop_time      # p02: first time the 0.5 s windowed true speed stays below 0.02 m/s (imports rclpy)
    rows = [r[:3] for r in gt]
    t_stop, _ = stop_time(rows, t_cancel)
    g0 = next(r for r in gt if r[0] >= t_cancel)
    gs = next((r for r in gt if t_stop and r[0] >= t_stop), gt[-1])
    return dict(stop_s=round(t_stop - t_cancel, 2) if t_stop else None,
                walked_after_cancel_m=round(math.hypot(gs[1] - g0[1], gs[2] - g0[2]), 3))


def odom_end_error(gt, odom, t):
    """|odometry - truth| at time t. The odom frame starts at A with heading 0 (where the robot spawns), so no alignment."""
    if not odom:
        return None
    o = min(odom, key=lambda r: abs(r[0] - t)); g = min(gt, key=lambda r: abs(r[0] - t))
    return round(math.hypot(o[1] - g[1], o[2] - g[2]), 3)
