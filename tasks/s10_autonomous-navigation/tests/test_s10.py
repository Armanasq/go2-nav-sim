"""Fast checks, no simulator:  python3 -m pytest -q tasks/s10_autonomous-navigation/tests"""
import json, math, pathlib, sys
from collections import deque
import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE / 'worlds'))
import make_worlds as mw
import score

RADIUS = 0.33                                         # robot_radius in nav2_params.yaml
NAMES = list(mw.WORLDS)


def free_space(w, radius=RADIUS):
    """Cells where the footprint circle fits (the map's occupied cells dilated by robot_radius)."""
    occ, origin = mw.occupancy(w)
    k = int(math.ceil(radius / mw.RES))
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
    disk = (xx ** 2 + yy ** 2) * mw.RES ** 2 <= radius ** 2
    pad = np.pad(occ, k, constant_values=True)
    grown = np.zeros_like(occ)
    for dy, dx in zip(*np.nonzero(disk)):
        grown |= pad[dy:dy + occ.shape[0], dx:dx + occ.shape[1]]
    return ~grown, origin


def reachable(w, start, goal):
    free, (ox, oy) = free_space(w)
    cell = lambda p: (int((p[1] - oy) / mw.RES), int((p[0] - ox) / mw.RES))
    s, g = cell(start), cell(goal)
    inside = lambda c: 0 <= c[0] < free.shape[0] and 0 <= c[1] < free.shape[1]
    if not inside(g) or not free[g] or not free[s]:
        return False
    seen, q = {s}, deque([s])
    while q:
        c = q.popleft()
        if c == g:
            return True
        for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (c[0] + d[0], c[1] + d[1])
            if inside(n) and free[n] and n not in seen:
                seen.add(n); q.append(n)
    return False


@pytest.mark.parametrize('name', NAMES)
def test_start_and_goal_are_clear(name):
    w = mw.WORLDS[name]
    assert mw.clearance(w, *mw.A) >= 0.6
    assert mw.clearance(w, *w['goal'][:2]) >= 0.6


@pytest.mark.parametrize('name', NAMES)
def test_goal_reachable_with_the_footprint(name):
    assert reachable(mw.WORLDS[name], mw.A, mw.WORLDS[name]['goal'])


def test_unreachable_goals_of_10_09():
    w = mw.WORLDS['one_obstacle']
    assert not reachable(w, mw.A, w['blocks'][0][:2])           # inside the box
    assert not reachable(w, mw.A, (8.0, 8.0))                   # outside the map


def test_waypoints_are_clear():
    import run
    for s in run.WAYPOINTS:
        x, y, _ = map(float, s.split(','))
        assert mw.clearance(mw.WORLDS['obstacles'], x, y) >= 0.6
        assert reachable(mw.WORLDS['obstacles'], mw.A, (x, y))


def test_passages_are_one_metre():
    """The narrow passage and the doors are 1.0 m: wider than the 0.66 m footprint, narrower than 2 footprints."""
    for name, probe in [('narrow_passage', (-2.5, -2.0)), ('doorway', (-2.5, -0.8)), ('rooms', (-2.0, -1.5)), ('rooms', (1.0, 0.0))]:
        assert mw.clearance(mw.WORLDS[name], *probe) == pytest.approx(0.5, abs=1e-9)


def test_map_matches_geometry():
    """A cell is occupied in the PGM exactly when its centre is inside an obstacle (spot check against clearance)."""
    w = mw.WORLDS['rooms']
    occ, (ox, oy) = mw.occupancy(w)
    rng = np.random.default_rng(0)
    for i, j in zip(rng.integers(0, occ.shape[0], 300), rng.integers(0, occ.shape[1], 300)):
        x, y = ox + (j + .5) * mw.RES, oy + (i + .5) * mw.RES
        inside = mw.clearance(w, x, y) == 0.0
        assert occ[i, j] == inside or abs(mw.clearance(w, x, y)) < 1e-9


def test_modified_world_blocks_old_path_only_partly():
    """10.15: the modified world differs from the mapped one, and B is still reachable in it."""
    assert mw.WORLDS['obstacles_mod']['blocks'] != mw.WORLDS['obstacles']['blocks']
    assert reachable(mw.WORLDS['obstacles_mod'], mw.A, mw.WORLDS['obstacles_mod']['goal'])


def test_path_length_ignores_sway():
    t = np.linspace(0, 10, 2001)
    xy = list(zip(t * 0.3, 0.01 * np.sin(2 * np.pi * 2 * t)))  # 3 m with 1 cm sway at 2 Hz
    assert score.path_length(xy) == pytest.approx(3.0, rel=0.02)          # sway adds 1.7 % here


def test_leg_scores():
    w = mw.WORLDS['room']
    gt = [(0.1 * i, -4.5 + 0.04 * i, -3.5 + 0.03 * i, 0.64) for i in range(101)]     # straight to (-0.5, -0.5)
    amcl = [(0.1 * i, -4.5 + 0.04 * i + 0.1, -3.5 + 0.03 * i, 0.0) for i in range(0, 101, 10)]
    s = score.leg(gt, [a[:3] for a in amcl], 0.0, 10.0, (-0.5, -0.5, 0.64), w)
    assert s['final_error_m'] == pytest.approx(0.0, abs=1e-6)
    assert s['path_m'] == pytest.approx(5.0, abs=0.01)
    assert s['amcl_rmse_m'] == pytest.approx(0.1, abs=1e-6)
    assert s['min_clearance_m'] == pytest.approx(mw.clearance(w, -0.5, -0.5), abs=1e-3)


def test_box_distance_and_replan():
    box = dict(x=0.0, y=0.0, yaw=math.pi / 2, t=5.0)
    size = (0.3, 1.6)                                           # rotated 90 deg: 1.6 m along x, 0.3 m along y
    assert score.box_dist([0.0], [0.0], box, size)[0] == 0.0
    assert score.box_dist([1.0], [0.0], box, size)[0] == pytest.approx(0.2)
    assert score.box_dist([0.0], [1.0], box, size)[0] == pytest.approx(0.85)
    through = np.c_[np.zeros(21), np.linspace(-2, 2, 21)]      # a plan straight through the box
    around = np.c_[np.r_[np.zeros(5), np.full(11, 1.5), np.zeros(5)], np.linspace(-2, 2, 21)]
    r = score.replan([(4.0, through), (6.0, through), (7.5, around)], [(5.0, 0, -2, 0), (8.0, 1.2, 0, 0)], box, size, 10.0)
    assert r['plan_blocked_at_spawn'] and r['replan_s'] == pytest.approx(2.5)


def test_acceptance_has_every_task():
    acc = json.loads((HERE / 'acceptance.json').read_text())
    assert [k for k in acc if not k.startswith('_')] == [f'10.{i:02d}' for i in range(1, 16)]
    for k, v in acc.items():
        if not k.startswith('_'):
            assert v['checks'] and v['runs'] >= 1
