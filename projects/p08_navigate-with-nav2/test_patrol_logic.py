import math
import pathlib
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import patrol_logic as pl  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'src/go2_nav_sim/scripts'))
import make_arena as arena  # noqa: E402

LINE = np.c_[np.zeros(11), np.linspace(0, 10, 11)]          # straight plan up the y axis, 1 m between points


def test_path_length():
    assert math.isclose(pl.path_length(LINE), 10.0)
    assert pl.path_length(LINE[:1]) == 0.0


def test_point_ahead_measures_along_the_plan_from_the_nearest_point():
    assert pl.point_ahead(LINE, (0.3, 2.2), 2.0) == (0.0, 4.0)
    assert pl.point_ahead(LINE, (0.0, 9.0), 5.0) == (0.0, 10.0)    # clamped to the end


def test_box_clearance():
    c = (4.0, 0.0)
    assert pl.box_clearance([[4.0, 0.0]], c) == 0.0                 # inside
    assert math.isclose(pl.box_clearance([[5.4, 0.0]], c), 1.0)     # 1 m east of the east face (x = 4.4)
    assert math.isclose(pl.box_clearance([[5.4, 1.4], [9, 9]], c), math.hypot(1, 1))   # off a corner, nearest point wins
    assert pl.box_clearance([[4.0, -3.0], [4.0, 3.0]], c) == 0.0    # a segment through the box, both ends outside


def test_first_clear_plan_skips_plans_before_the_spawn_and_through_the_box():
    through, around = np.array([[4.0, -2.0], [4.0, 2.0]]), np.array([[5.0, -2.0], [5.0, 2.0]])
    plans = [(9.0, around), (10.5, through), (11.5, around)]
    assert pl.first_clear_plan(plans, 10.0, (4.0, 0.0)) == 2
    assert pl.first_clear_plan(plans[:2], 10.0, (4.0, 0.0)) is None


def test_box_sdf_is_valid_xml_with_the_right_size():
    m = ET.fromstring(pl.box_sdf('surprise_box')).find('model')
    assert m.get('name') == 'surprise_box' and m.find('static').text == 'true'
    assert m.find('link/collision/geometry/box/size').text == f'{pl.BOX} {pl.BOX} {pl.BOX}'


def test_arrival_yaws_face_the_way_the_robot_came():
    w = pl.arrival_yaws([(1.0, 0.0), (1.0, 2.0), (0.0, 2.0)], start=(0.0, 0.0))
    assert [round(y, 6) for _, _, y in w] == [0.0, round(math.pi / 2, 6), round(math.pi, 6)]


def test_leg_times():
    assert pl.leg_times(10.0, [40.0, 75.5, 100.0], 140.2) == [30.0, 35.5, 24.5, 40.2]


def test_waypoints_are_clear_of_the_arena_obstacles():
    for x, y, _ in pl.WAYPOINTS:
        for cx, cy, sx, sy in [w[:4] for w in arena.WALLS] + [b[:4] for b in arena.BOXES]:
            gap = math.hypot(max(abs(x - cx) - sx / 2, 0), max(abs(y - cy) - sy / 2, 0))
            assert gap > pl.ROBOT_RADIUS + 0.5, (x, y, cx, cy)
        for cx, cy, r, *_ in arena.CYLS:
            assert math.hypot(x - cx, y - cy) - r > pl.ROBOT_RADIUS + 0.5
