"""Checks on the p11 configs and on recorded message logs. No ROS: runs under plain python3 and pytest.
usage: python3 projects/p11_command-arbitration/arbitration_check.py      prints the zone geometry and the mux order"""
import pathlib
import yaml

HERE = pathlib.Path(__file__).resolve().parent
BODY_FRONT, BODY_HALF_WIDTH = 0.325, 0.15      # Go2 body 0.65 x 0.30 m, centred on base_footprint (chapter 10)
SCAN_RANGE_MIN = 0.40                           # src/go2_nav_sim/config/scan.yaml


def load(name):
    cfg = yaml.safe_load((HERE / 'config' / name).read_text())
    assert isinstance(cfg, dict)
    return next(iter(cfg.values()))['ros__parameters']


def polygon(params, name):
    p = params[name]['points']
    return list(zip(p[0::2], p[1::2]))


def box_extent(pts):
    xs, ys = zip(*pts)
    return min(xs), max(xs), min(ys), max(ys)


def first_after(log, t0, pred):
    """First (t, value) in a time-sorted log with t >= t0 whose value satisfies pred; None if there is none."""
    return next(((t, v) for t, v in log if t >= t0 and pred(v)), None)


def is_key(vx, wz, key_wz=1.0, ratio=0.3):
    """True for the 'j' key command (vx 0, wz 1.0 rad/s) as it leaves the mux, or as the slowdown zone scales it (wz * ratio)."""
    return abs(vx) < 1e-3 and min(abs(wz - key_wz), abs(wz - key_wz * ratio)) < 1e-3


def stop_gap(robot_x, box_face_x, body_front=BODY_FRONT):
    """Free space between the front of the body and the box face, both on the walking axis."""
    return box_face_x - (robot_x + body_front)


if __name__ == '__main__':
    cm = load('collision_monitor.yaml')
    for name in cm['polygons']:
        x0, x1, y0, y1 = box_extent(polygon(cm, name))
        print(f'{name}: x {x0:+.2f}..{x1:+.2f}  y {y0:+.2f}..{y1:+.2f}  '
              f'ahead of the body {x1 - BODY_FRONT:.3f} m, beside it {y1 - BODY_HALF_WIDTH:.3f} m, '
              f'visible depth {x1 - SCAN_RANGE_MIN:.2f} m')
    mux = load('twist_mux.yaml')
    order = sorted(((v['priority'], k, v['topic']) for k, v in {**mux['topics'], **mux['locks']}.items()), reverse=True)
    print('mux order:', ', '.join(f'{k} ({t}) {p}' for p, k, t in order))
