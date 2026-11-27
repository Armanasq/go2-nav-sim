"""The Gazebo world and the occupancy map are generated from one list; check they agree."""
import importlib.util, pathlib
import numpy as np

PKG = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('arena', PKG / 'scripts/make_arena.py'); arena = importlib.util.module_from_spec(spec); spec.loader.exec_module(arena)


def load_map():
    meta = dict(l.split(': ', 1) for l in (PKG / 'maps/arena.yaml').read_text().splitlines() if ': ' in l)
    raw = (PKG / 'maps' / meta['image']).read_bytes(); _, dims, _, body = raw.split(b'\n', 3); w, h = map(int, dims.split())
    img = np.frombuffer(body, np.uint8).reshape(h, w)
    ox, oy = (float(v) for v in meta['origin'].strip('[]').split(',')[:2]); res = float(meta['resolution'])
    return lambda x, y: img[h - 1 - int((y - oy) / res), int((x - ox) / res)]


def test_obstacles_are_occupied_and_markers_free():
    cell = load_map()
    for x, y, *_ in arena.BOXES + arena.CYLS:
        assert cell(x, y) == 0, f'obstacle at ({x}, {y}) missing from the map'
    for x, y in (arena.A, arena.B):
        assert cell(x, y) == 254, f'marker ({x}, {y}) is not free'


def test_world_contains_every_obstacle():
    world = (PKG / 'worlds/arena.world').read_text()
    assert world.count('<model name="box') == len(arena.BOXES)
    assert world.count('<model name="pillar') == len(arena.CYLS)
    assert world.count('<model name="wall') == len(arena.WALLS)


def test_gaps_are_wide_enough_for_the_footprint():
    """Every pair of obstacles leaves more than 2 x 0.33 m (footprint radius) of clearance, or touches a wall."""
    def boxes():
        for x, y, sx, sy, *_ in arena.BOXES:
            yield x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2
        for x, y, r, *_ in arena.CYLS:
            yield x - r, x + r, y - r, y + r
    b = list(boxes())
    for i in range(len(b)):
        for j in range(i + 1, len(b)):
            dx = max(b[j][0] - b[i][1], b[i][0] - b[j][1], 0); dy = max(b[j][2] - b[i][3], b[i][2] - b[j][3], 0)
            assert np.hypot(dx, dy) > 0.66, f'gap between obstacles {i} and {j} is {np.hypot(dx, dy):.2f} m'
