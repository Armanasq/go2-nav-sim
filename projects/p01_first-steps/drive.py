#!/usr/bin/env python3
"""Walk a timed sequence of velocity commands, then stop.
usage: python3 projects/p01_first-steps/drive.py [vx,vy,wz,seconds ...]
Default: forward 0.3 m/s for 10 s, turn left 0.5 rad/s for 3.2 s, forward 0.3 m/s for 3 s.
Durations are simulated seconds. /cmd_vel is published at 20 Hz, and a zero twist is always sent at the end, also on Ctrl-C,
because CHAMP keeps walking on the last command it received."""
import sys

DEFAULT = [(0.3, 0.0, 0.0, 10.0), (0.0, 0.0, 0.5, 3.2), (0.3, 0.0, 0.0, 3.0)]


def parse(args):
    """['0.3,0,0,10', ...] -> [(0.3, 0.0, 0.0, 10.0), ...]"""
    segs = []
    for a in args:
        vals = [float(v) for v in a.split(',')]
        if len(vals) != 4 or vals[3] < 0:
            raise ValueError(f'expected vx,vy,wz,seconds with seconds >= 0, got {a!r}')
        segs.append(tuple(vals))
    return segs


def command_at(segs, t):
    """The (vx, vy, wz) to send t seconds after the start, or None once the sequence is over."""
    for vx, vy, wz, dur in segs:
        if t < dur:
            return vx, vy, wz
        t -= dur
    return None


def main():
    import signal, time
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist

    segs = parse(sys.argv[1:]) or DEFAULT
    # rclpy's own Ctrl-C handler shuts the context down, and then the final zero twist cannot be published. Use Python's.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    node = rclpy.create_node('drive', parameter_overrides=[Parameter('use_sim_time', value=True)])
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    while rclpy.ok() and node.get_clock().now().nanoseconds == 0:      # no /clock yet: sim time reads 0
        rclpy.spin_once(node, timeout_sec=0.1)
    t0 = node.get_clock().now()
    last = None

    def tick():
        nonlocal last
        cmd = command_at(segs, (node.get_clock().now() - t0).nanoseconds * 1e-9)
        if cmd is None:
            raise SystemExit
        if cmd != last:
            print(f'{(node.get_clock().now() - t0).nanoseconds * 1e-9:5.1f} s  vx {cmd[0]:.2f}  vy {cmd[1]:.2f}  wz {cmd[2]:.2f}', flush=True)
            last = cmd
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = cmd
        pub.publish(m)

    node.create_timer(0.05, tick)              # 20 Hz on the simulated clock
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        for _ in range(3):                     # zero twist; repeated so a single dropped message cannot leave it walking
            pub.publish(Twist())
            time.sleep(0.05)
        print('stop: zero twist sent', flush=True)
        time.sleep(0.3)                        # let DDS deliver before the process exits
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
