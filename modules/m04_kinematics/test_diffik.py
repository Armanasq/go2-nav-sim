"""Module 04 advanced level: the active-set QP and constrained differential IK, same thresholds as validate.py."""
import pathlib, sys
import numpy as np
import pytest
mujoco = pytest.importorskip('mujoco')
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1])); sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from common import GO2_XML  # noqa: E402
from scipy.optimize import lsq_linear  # noqa: E402
from kinematics import Leg  # noqa: E402
from diffik import QD_MAX, box, distance_to_workspace, qp_active_set, reach_radius, track  # noqa: E402
import importlib.util  # noqa: E402
# Loaded by path under its own name: every module has a validate.py, and `import validate` would reuse whichever was imported first.
_spec = importlib.util.spec_from_file_location('m04_validate', pathlib.Path(__file__).resolve().parent / 'validate.py')
V = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(V)

pytestmark = pytest.mark.skipif(not GO2_XML.exists(), reason='run modules/fetch_go2.sh first')


@pytest.fixture(scope='module')
def leg():
    return Leg(mujoco.MjModel.from_xml_path(str(GO2_XML)), 'FL')


def test_active_set_matches_bvls_and_kkt():
    r = V.qp_checks(300)
    assert r['qp_vs_lsq_linear_max'] < 1e-9 and r['qp_kkt_residual_max'] < 1e-9 and r['qp_max_iterations_random'] <= 10


def test_active_set_general_constraints():
    # min (x0 - 1)^2 + (x1 - 2.5)^2 under five constraints (Nocedal and Wright, ch. 16); solution (1.4, 1.7) by hand
    H, g = 2 * np.eye(2), np.array([-2.0, -5.0])
    A = np.array([[-1, 2], [1, 2], [1, -2], [-1, 0], [0, -1]], float); b = np.array([2, 6, 2, 0, 0], float)
    x, y, _ = qp_active_set(H, g, A, b, np.array([2.0, 0.0]))
    assert np.allclose(x, [1.4, 1.7], atol=1e-12) and np.all(y >= 0) and np.allclose(H @ x + g + A.T @ y, 0, atol=1e-12)


def test_reach_radius_and_distance(leg):
    assert abs(reach_radius(leg) - 0.4015107) < 1e-6
    p = leg.hip + np.array([0.0, leg.d, -0.45])                       # straight below the thigh joint, beyond reach
    assert abs(distance_to_workspace(leg, p) - (np.hypot(0.45, leg.d) - reach_radius(leg))) < 1e-15
    S = leg.ik_closed_form(leg.hip + np.array([0.0, leg.d, -0.38]))   # inside: closed form finds a solution
    assert len(S) and distance_to_workspace(leg, leg.hip + np.array([0.0, leg.d, -0.38])) == 0


def test_boundary_path_qp_vs_dls(leg):
    t, p, q0 = V.traj_boundary(leg, cycles=1); dist = distance_to_workspace(leg, p)
    qp, dls = track(leg, p, q0, V.DT, 'qp', lam=V.LAM), track(leg, p, q0, V.DT, 'dls', lam=V.LAM)
    e_qp = np.linalg.norm(qp['p'] - p, axis=1)
    assert np.all(qp['q'] <= leg.hi + 1e-12) and np.all(qp['q'] >= leg.lo - 1e-12)
    assert (e_qp - dist).max() < 1e-3 and np.sqrt(np.mean(e_qp[dist == 0] ** 2)) < 1e-4
    assert dls['q'][:, 2].max() > leg.hi[2] + 0.3                     # unconstrained DLS straightens the knee past its limit


def test_singular_path_qp_respects_speed_limits(leg):
    t, p, q0 = V.traj_singular(leg, cycles=1); dist = distance_to_workspace(leg, p)
    qp, dls = track(leg, p, q0, V.DT, 'qp', lam=V.LAM), track(leg, p, q0, V.DT, 'dls', lam=V.LAM)
    sp = lambda r: np.abs(np.diff(np.vstack([q0, r['q']]), axis=0)) / V.DT    # noqa: E731
    e_qp = np.linalg.norm(qp['p'] - p, axis=1)
    assert (sp(qp) / QD_MAX).max() <= 1 + 1e-9 and (sp(dls) / QD_MAX).max() > 2
    assert np.sqrt(np.mean(e_qp[dist == 0] ** 2)) < 1e-3 and (e_qp - dist).max() < 5e-3


def test_figure_eight_in_mujoco():
    r, _ = V.run_figure_eight(duration=3.0)
    assert r['fig8_ik_max_error_m'] < 1e-4 and r['fig8_mujoco_rms_error_m'] < 5e-3 and r['fig8_mujoco_max_error_m'] < 1e-2
