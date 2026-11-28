"""Fast checks of urdf_fk (no ROS, no simulator):  /usr/bin/python3 -m pytest -q tests"""
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from urdf_fk import Urdf, axis_angle, quat_matrix, rotation_angle, rpy_matrix  # noqa: E402

LEG = """<robot name="leg">
  <link name="base"/>
  <link name="thigh"><inertial><origin xyz="0 0 -0.1"/><mass value="2.0"/></inertial></link>
  <link name="foot"/>
  <joint name="hip" type="revolute"><parent link="base"/><child link="thigh"/><origin xyz="0.2 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
  <joint name="ankle" type="fixed"><parent link="thigh"/><child link="foot"/><origin xyz="0 0 -0.3"/></joint>
</robot>"""


def test_rpy_and_axis_angle_agree():
    for r, p, y in [(0.3, 0, 0), (0, -0.7, 0), (0, 0, 2.0)]:
        R = rpy_matrix(r, p, y)
        a = [(1, 0, 0), (0, 1, 0), (0, 0, 1)][[r, p, y].index(next(v for v in (r, p, y) if v))]
        assert np.allclose(R, axis_angle(a, next(v for v in (r, p, y) if v)))
    assert np.allclose(rpy_matrix(0, 0, np.pi), np.diag([-1, -1, 1]))


def test_quaternion_and_angle():
    assert np.allclose(quat_matrix(0, 0, 1, 0), np.diag([-1, -1, 1]))          # yaw pi, as tf2_echo prints for rear_cam_link
    assert rotation_angle(axis_angle((0, 1, 0), 0.4)) == pytest.approx(0.4)


def test_fk_pendulum():
    u = Urdf(LEG)
    assert u.root == 'base' and u.actuated() == ['hip'] and u.chain('foot') == ['hip', 'ankle']
    assert np.allclose(u.fk({}, 'foot')[:3, 3], [0.2, 0.1, -0.3])
    q = 0.5                                                  # +y rotation moves a point below the axis towards -x
    assert np.allclose(u.fk({'hip': q}, 'foot')[:3, 3], [0.2 - 0.3 * np.sin(q), 0.1, -0.3 * np.cos(q)])
    assert np.allclose(u.fk({'hip': q}, 'foot', base='thigh')[:3, 3], [0, 0, -0.3])


def test_holding_effort_pendulum():
    u = Urdf(LEG)
    assert u.holding_effort({}, 'hip') == pytest.approx(0.0, abs=1e-12)        # hanging straight down
    q = 0.5                  # com at 0.1 m swung towards -x: gravity torque about +y is -m g l sin q, the joint must supply +m g l sin q
    assert u.holding_effort({'hip': q}, 'hip') == pytest.approx(2.0 * 9.8 * 0.1 * np.sin(q))


def test_single_root_required():
    with pytest.raises(ValueError):
        Urdf('<robot name="r"><link name="a"/><link name="b"/></robot>')
