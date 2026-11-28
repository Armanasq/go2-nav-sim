"""Go-to-pose with the polar-coordinate controller. No ROS in this file, so it can be unit-tested.

Reference: R. Siegwart, I. Nourbakhsh, D. Scaramuzza, Introduction to Autonomous Mobile Robots, 2nd ed., section 3.6.2.4.
  rho   distance from the robot to the goal position
  alpha angle from the robot's heading to the line robot -> goal
  beta  angle from that line to the goal heading
  v = k_rho * rho,  w = k_alpha * alpha + k_beta * beta
Locally exponentially stable when k_rho > 0, k_beta < 0, k_alpha - k_rho > 0 (strong_stability() is the stricter test)."""
import math

V_MAX, W_MAX = 0.3, 0.5                 # CHAMP limits for the Go2: m/s forward, rad/s
GAINS = (0.5, 1.5, -0.5)                # k_rho, k_alpha, k_beta used for the results in the README
HIGH_GAINS = (3.0, 8.0, -1.5)           # the textbook example gains, for the too-high-gain experiment


def wrap(a):
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def polar(x, y, yaw, gx, gy, gyaw):
    """Robot pose (x, y, yaw) and goal pose (gx, gy, gyaw) in one frame -> (rho, alpha, beta)."""
    rho = math.hypot(gx - x, gy - y)
    bearing = math.atan2(gy - y, gx - x)
    return rho, wrap(bearing - yaw), wrap(gyaw - bearing)


def strong_stability(k_rho, k_alpha, k_beta):
    """Siegwart's condition for the robot to reach the goal without changing direction of travel."""
    return k_rho > 0 and k_beta < 0 and k_alpha + 5 / 3 * k_beta - 2 / math.pi * k_rho > 0


def saturate(v, w, v_max=V_MAX, w_max=W_MAX):
    """Scale v and w by the same factor so both fit the limits. Same ratio w/v = same path, only slower."""
    s = max(1.0, abs(v) / v_max, abs(w) / w_max)
    return v / s, w / s


class GoToPose:
    """step(x, y, yaw) -> (v, w, phase). Phase 'drive' runs the polar law until the robot is within pos_tol of the goal,
    'align' turns on the spot to the goal heading, 'done' when the heading is within yaw_tol."""

    def __init__(self, goal, gains=GAINS, pos_tol=0.05, yaw_tol=math.radians(3), v_min=0.1, w_min=0.1):
        self.goal, self.k, self.pos_tol, self.yaw_tol = goal, gains, pos_tol, yaw_tol
        self.v_min, self.w_min = v_min, w_min
        self.phase, self.direction = 'drive', None

    def step(self, x, y, yaw):
        gx, gy, gyaw = self.goal
        if self.phase == 'drive':
            rho, alpha, _ = polar(x, y, yaw, gx, gy, gyaw)
            if self.direction is None:            # chosen once per goal: forward if the goal is in front, else backward
                self.direction = 1 if abs(alpha) <= math.pi / 2 else -1
            if rho < self.pos_tol:
                self.phase = 'align'
            else:
                flip = 0.0 if self.direction > 0 else math.pi    # walking backward = forward with the heading flipped
                rho, alpha, beta = polar(x, y, yaw + flip, gx, gy, gyaw + flip)
                v, w = self.direction * self.k[0] * rho, self.k[1] * alpha + self.k[2] * beta
                up = max(1.0, self.v_min / abs(v))   # CHAMP barely walks below 0.1 m/s: scale up, same ratio w/v
                v, w = saturate(v * up, w * up)
                return v, w, 'drive'
        if self.phase == 'align':
            err = wrap(gyaw - yaw)
            if abs(err) < self.yaw_tol:
                self.phase = 'done'
            else:
                w = max(-W_MAX, min(W_MAX, 1.5 * err))
                return 0.0, math.copysign(max(abs(w), self.w_min), err), 'align'
        return 0.0, 0.0, self.phase                      # 'done', or 'timeout' when the node gave up
