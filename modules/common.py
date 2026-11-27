"""Shared helpers for the modules: where the Go2 model is, and small SO(3)/SE(3) utilities written out explicitly."""
import pathlib
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
GO2_XML = HERE / 'assets/unitree_go2/go2.xml'
LEGS = ('FL', 'FR', 'RL', 'RR')


def require_model():
    if not GO2_XML.exists():
        raise SystemExit('Go2 model missing: run modules/fetch_go2.sh first')
    return str(GO2_XML)


def quat_to_R(q):
    """Unit quaternion (w, x, y, z), MuJoCo's order, to a rotation matrix."""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def axis_angle_R(a, th):
    """Rodrigues' formula: rotation by th about the unit axis a."""
    a = np.asarray(a, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
