"""Velocity limiter with a dead-man timeout. Plain Python, no ROS, so it can be unit-tested on its own.

Commands are (vx, vy, wz): forward m/s, sideways m/s, yaw rad/s, in the robot's base frame.
"""


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Limiter:
    def __init__(self, v_max=(0.3, 0.25, 0.5), a_max=(0.5, 0.5, 1.0), timeout=0.6):
        self.v_max, self.a_max, self.timeout = v_max, a_max, timeout
        self.target = (0.0, 0.0, 0.0)      # last command, after the velocity clip
        self.out = (0.0, 0.0, 0.0)         # what was last sent to the robot
        self.t_cmd = None                  # time of the last command; None = never received one
        self.t_out = None                  # time of the last output
        self.zeros_sent = 0

    def command(self, t, cmd):
        """A new command arrived at time t (seconds)."""
        self.target = tuple(clamp(c, -m, m) for c, m in zip(cmd, self.v_max))
        self.t_cmd = t

    def stale(self, t):
        # t < t_cmd: the clock went back (the simulator was restarted), so the last command belongs to another run
        return self.t_cmd is None or t < self.t_cmd or t - self.t_cmd > self.timeout

    def step(self, t):
        """Output for time t, or None when there is nothing to send (idle and already stopped)."""
        target = (0.0, 0.0, 0.0) if self.stale(t) else self.target
        dt = 0.0 if self.t_out is None else max(0.0, t - self.t_out)
        self.t_out = t
        # move each axis towards its target by at most a_max * dt
        self.out = tuple(o + clamp(g - o, -a * dt, a * dt) for o, g, a in zip(self.out, target, self.a_max))
        if self.stale(t) and self.out == (0.0, 0.0, 0.0):
            self.zeros_sent += 1
            # ponytail: a few zeros, then silence, so a stopped limiter never fights another /cmd_vel publisher (Nav2)
            return self.out if self.zeros_sent <= 5 else None
        self.zeros_sent = 0
        return self.out
