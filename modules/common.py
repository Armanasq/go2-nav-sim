"""Shared helpers for the modules: where the Go2 model is, and small SO(3)/SE(3) utilities written out explicitly."""
import pathlib
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
GO2_XML = HERE / 'assets/unitree_go2/go2.xml'
SCENE_XML = HERE / 'assets/unitree_go2/scene.xml'      # go2.xml + floor plane, light, skybox
LEGS = ('FL', 'FR', 'RL', 'RR')
JOINTS = ('hip', 'thigh', 'calf')                      # per leg, in qpos/actuator order
G = 9.81                                               # m/s^2, MuJoCo's default |opt.gravity|


def require_model(scene=False):
    path = SCENE_XML if scene else GO2_XML
    if not path.exists():
        raise SystemExit('Go2 model missing: run modules/fetch_go2.sh first')
    return str(path)


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
