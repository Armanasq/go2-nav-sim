import math, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from tuning import (INSCRIBED, LETHAL, body_clearance, distance_grid, inflation_cost, load_map, lookup, parse_mission,
                    path_length, perimeter)

ROOT = pathlib.Path(__file__).resolve().parents[3]


def test_inflation_cost_matches_chapter_10():
    # chapter 10: 252 * exp(-4.0 * (0.425 - 0.33)) is about 172 at the centre of a 0.85 m gap
    assert inflation_cost(0.425, 0.33, 0.75, 4.0) == 172
    assert inflation_cost(0.0, 0.33, 0.75, 4.0) == LETHAL
    assert inflation_cost(0.30, 0.33, 0.75, 4.0) == INSCRIBED
    assert inflation_cost(0.76, 0.33, 0.75, 4.0) == 0


def test_higher_scale_decays_faster():
    d = 0.5
    assert inflation_cost(d, 0.33, 0.75, 10.0) < inflation_cost(d, 0.33, 0.75, 4.0) < inflation_cost(d, 0.33, 0.75, 1.0)


def test_distance_grid_and_lookup():
    occ = np.zeros((20, 20), bool); occ[:, 10] = True                 # a wall along x = 10 cells
    dist = distance_grid(occ, 0.1)
    assert lookup(dist, (0, 0), 0.1, [(0.55, 1.0)])[0] == 0.5          # cell 5 -> 5 cells from cell 10
    assert lookup(dist, (0, 0), 0.1, [(-1.0, 1.0)])[0] == 0.0          # outside counts as blocked


def test_perimeter_closed_and_spaced():
    sq = [[1, 1], [1, -1], [-1, -1], [-1, 1]]
    p = perimeter(sq, step=0.5)
    assert len(p) == 16 and np.allclose(np.abs(p).max(1), 1.0)


def test_body_clearance_uses_yaw():
    occ = np.zeros((100, 100), bool); occ[:, 60] = True               # wall at x = 3.0 m (cell centre 3.025)
    dist = distance_grid(occ, 0.05)
    body = [[0.325, 0.15], [0.325, -0.15], [-0.325, -0.15], [-0.325, 0.15]]
    along = body_clearance([(2.0, 2.5, 0.0)], body, dist, (0, 0), 0.05)          # nose towards the wall
    across = body_clearance([(2.0, 2.5, math.pi / 2)], body, dist, (0, 0), 0.05)  # side towards the wall
    assert abs(along - (3.0 - 2.325)) < 0.06 and abs(across - (3.0 - 2.15)) < 0.06 and across > along


def test_path_length():
    assert path_length([(0, 0), (3, 4), (3, 5)]) == 6.0 and path_length([(1, 1)]) == 0.0


def test_parse_mission():
    log = ('[INFO] [basic_navigator]: distance remaining 3.10 m, recoveries 0\n'
           '[INFO] [basic_navigator]: distance remaining 1.00 m, recoveries 1\n'
           'RESULT SUCCEEDED  sim_time 54.4 s  ground-truth final error 0.25 m  final pos (3.80, 2.85)\n')
    assert parse_mission(log) == dict(result='SUCCEEDED', nav_time_s=54.4, recoveries=1)
    assert parse_mission('RESULT TIMEOUT (600 s wall clock)')['result'] == 'TIMEOUT'


def test_arena_map_orientation():
    occ, origin, res = load_map(ROOT / 'src/go2_nav_sim/maps/arena.yaml')
    dist = distance_grid(occ, res)
    # the brick box centred at (1.0, 0.0) is 0.8 x 2.4 m: its centre is an obstacle, A (-4.5, -3.5) is free
    assert lookup(dist, origin, res, [(1.0, 0.0)])[0] == 0.0
    assert lookup(dist, origin, res, [(-4.5, -3.5)])[0] > 0.8
    # box at (2.5, -2.5) is 1.2 x 1.2 m; the point (2.5, -1.4) is 0.5 m north of it (but the map adds no margin)
    assert abs(lookup(dist, origin, res, [(2.5, -1.4)])[0] - 0.5) < 0.06


def test_make_params_changes_only_what_the_setting_names():
    import yaml
    from make_params import BASE, SETTINGS, make
    base = yaml.safe_load(BASE.read_text())
    g = lambda P, cm: P[cm][cm]['ros__parameters']
    P = make(base, *SETTINGS['polygon'])
    for cm in ('local_costmap', 'global_costmap'):                    # a footprint replaces robot_radius in both costmaps
        assert 'robot_radius' not in g(P, cm) and g(P, cm)['footprint'].startswith('[[0.325')
    assert 'robot_radius' in g(base, 'global_costmap')                # the stack's dict is not modified
    R = make(base, *SETTINGS['infl_r150'])
    assert g(R, 'global_costmap')['inflation_layer']['inflation_radius'] == 1.5 and g(R, 'local_costmap')['robot_radius'] == 0.33
    fp = lambda P: P['controller_server']['ros__parameters']['FollowPath']
    N = make(base, *SETTINGS['polygon_norot'])                         # a dict without 'plugin' is merged into RPP's block
    assert fp(N)['use_rotate_to_heading'] is False and fp(N)['plugin'] == fp(base)['plugin'] and fp(N)['lookahead_dist'] == 0.6
    assert fp(make(base, *SETTINGS['smac_mppi']))['plugin'] == 'nav2_mppi_controller::MPPIController'


def test_polygon_costmap_includes_padding():
    # the recorded polygon costmap fits inscribed 0.15 m + footprint_padding 0.01 m, not the bare 0.15 m half-width
    d = np.load(pathlib.Path(__file__).resolve().parents[1] / 'results/polygon_1.npz'); c = d['costmap']
    dist = distance_grid(c == LETHAL, float(d['res'])); m = (c > 0) & (c < INSCRIBED)
    err = lambda ins: np.mean(np.abs(np.array([inflation_cost(x, ins, 0.75, 4.0) for x in dist[m]]) - c[m]))
    assert err(0.16) < 1.5 < 4.0 < err(0.15)
