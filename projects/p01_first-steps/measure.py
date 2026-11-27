#!/usr/bin/env python3
"""Measure what the robot did, from ground truth (/odom_gt) and the estimate (/odom).
usage: python3 projects/p01_first-steps/measure.py [--duration S] [--csv run.csv]
Prints the true pose once per simulated second, and a summary on exit (Ctrl-C, SIGTERM, or after --duration s of sim time).
Poses are printed in the world (arena) frame: both topics start at (0, 0) on marker A, so A is added back."""
import argparse, math, signal, sys

A = (-4.5, -3.5)          # start marker in the world frame (src/go2_nav_sim/scripts/make_arena.py)


def yaw_of(q):
    """Heading (rad) of a quaternion with x, y, z, w fields: rotation about the vertical axis."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """Angle into [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def summarize(samples):
    """samples: list of (t, x, y, yaw). Returns duration, path length, net displacement, mean speed, total turn."""
    if len(samples) < 2:
        return None
    path = turn = 0.0
    for (_, x0, y0, a0), (_, x1, y1, a1) in zip(samples, samples[1:]):
        path += math.hypot(x1 - x0, y1 - y0)
        turn += wrap(a1 - a0)                  # sum of small steps, so a 400 degree spin counts as 400, not 40
    t0, x0, y0, _ = samples[0]
    t1, x1, y1, a1 = samples[-1]
    dur = t1 - t0
    return dict(duration=dur, path=path, displacement=math.hypot(x1 - x0, y1 - y0),
                speed=path / dur if dur > 0 else 0.0, turn_deg=math.degrees(turn),
                x=x1, y=y1, yaw_deg=math.degrees(wrap(a1)))


def report(name, s):
    if s is None:
        return f'{name}: no data'
    return (f"{name}: {s['duration']:.1f} s  path {s['path']:.2f} m  net {s['displacement']:.2f} m  "
            f"mean speed {s['speed']:.3f} m/s  turned {s['turn_deg']:.1f} deg  "
            f"end ({s['x']:.2f}, {s['y']:.2f}) heading {s['yaw_deg']:.1f} deg")


def main():
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import Odometry

    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=0.0, help='stop after this many seconds of sim time (0 = until Ctrl-C)')
    ap.add_argument('--csv', help='write every sample to this file, for plot_run.py')
    a = ap.parse_args()

    # rclpy's own Ctrl-C handler shuts the context down, and a message taken at that moment raised RuntimeError instead of
    # KeyboardInterrupt (1 of 5 Ctrl-Cs on a loaded machine), losing the summary and the CSV. Use Python's handler.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('measure', parameter_overrides=[Parameter('use_sim_time', value=True)])
    data = {'gt': [], 'odom': []}

    def on_odom(key):
        def cb(m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p = m.pose.pose
            data[key].append((t, p.position.x + A[0], p.position.y + A[1], yaw_of(p.orientation)))
        return cb

    node.create_subscription(Odometry, 'odom_gt', on_odom('gt'), 50)
    node.create_subscription(Odometry, 'odom', on_odom('odom'), 50)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))

    next_print = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            gt = data['gt']
            if not gt:
                continue
            t, x, y, yaw = gt[-1]
            next_print = t if next_print is None else next_print
            if t >= next_print:                # once per simulated second
                old = [s for s in gt if s[0] >= t - 1.0][0]
                v = math.hypot(x - old[1], y - old[2]) / max(t - old[0], 1e-6)
                print(f't {t - gt[0][0]:6.1f} s  x {x:6.2f}  y {y:6.2f}  heading {math.degrees(yaw):7.1f} deg  '
                      f'speed {v:.2f} m/s', flush=True)
                next_print += 1.0
            if a.duration and t - gt[0][0] >= a.duration:
                break
    except KeyboardInterrupt:
        pass

    print(report('ground truth', summarize(data['gt'])))
    print(report('estimate    ', summarize(data['odom'])))
    if a.csv:
        with open(a.csv, 'w') as f:
            f.write('source,t,x,y,yaw\n')
            for key, rows in data.items():
                f.writelines(f'{key},{t:.3f},{x:.4f},{y:.4f},{yaw:.5f}\n' for t, x, y, yaw in rows)
        print(f'wrote {a.csv}')
    node.destroy_node()


if __name__ == '__main__':
    main()
