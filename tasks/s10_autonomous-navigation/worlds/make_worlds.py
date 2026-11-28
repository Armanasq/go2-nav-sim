#!/usr/bin/env python3
"""Write the stage-10 Gazebo worlds and their occupancy maps from one geometry table (the method of make_arena.py).
usage: python3 tasks/s10_autonomous-navigation/worlds/make_worlds.py
Every map here is written from the known geometry, not built with slam_toolbox, so map and world agree exactly.
The robot always spawns on marker A = (-4.5, -3.5) facing +x (sim.launch.py), so every world is laid out around A."""
import math, pathlib, sys
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
PKG = HERE.parents[2] / 'src/go2_nav_sim'
sys.path.insert(0, str(PKG / 'scripts'))
from make_arena import box_model, cyl_model, marker, camera, mat      # reuse the arena's SDF helpers

A = (-4.5, -3.5)
RES, MARGIN, WALL_T, WALL_H, OBS_H = 0.05, 0.5, 0.2, 1.0, 0.8
DOOR = 1.0                                     # every door and passage: 1.0 m (the 0.33 m footprint needs > 0.66 m)


def walls(x0, x1, y0, y1):
    """Four walls centred on the room bounds, like the arena: inner faces 0.1 m inside the bounds."""
    cx, cy, w, h = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    return [(cx, y1, w + WALL_T, WALL_T), (cx, y0, w + WALL_T, WALL_T), (x1, cy, WALL_T, h), (x0, cy, WALL_T, h)]


def wall_x(x, y0, y1, door, t=WALL_T):
    """Wall along y at x = x from y0 to y1 with a DOOR-wide gap centred at y = door."""
    a, b = door - DOOR / 2, door + DOOR / 2
    return [(x, (y0 + a) / 2, t, a - y0), (x, (b + y1) / 2, t, y1 - b)]


def wall_y(y, x0, x1, door, t=WALL_T):
    a, b = door - DOOR / 2, door + DOOR / 2
    return [((x0 + a) / 2, y, a - x0, t), ((b + x1) / 2, y, x1 - b, t)]


ROOM = (-6.0, 1.0, -5.0, 1.0)                  # 7 x 6 m room shared by several worlds
B = (-0.5, -0.5, round(math.atan2(3.0, 4.0), 4))
# name: bounds, blocks (x, y, sx, sy), cylinders (x, y, r), goal (x, y, yaw), spectator camera
WORLDS = {
    'room':           dict(bounds=ROOM, blocks=[], cyls=[], goal=B, cam=True),
    'one_obstacle':   dict(bounds=ROOM, blocks=[(-2.5, -2.0, 1.0, 1.0)], cyls=[], goal=B),
    'obstacles':      dict(bounds=ROOM, blocks=[(-3.2, -2.1, 0.8, 0.8), (-1.3, -1.6, 0.6, 1.2)],
                           cyls=[(-1.9, -3.4, 0.35), (-2.7, -0.4, 0.4)], goal=B, cam=True),
    # same room, two obstacles moved and one added on the path the robot took in 'obstacles'; the map stays obstacles.yaml
    'obstacles_mod':  dict(bounds=ROOM, blocks=[(-3.2, -2.1, 0.8, 0.8), (-1.5, -0.2, 0.6, 1.2), (-2.4, -2.9, 0.8, 0.5)],
                           cyls=[(-1.6, -2.2, 0.35), (-2.7, -0.4, 0.4)], goal=B),
    # a 1.6 m thick partition with a 1.0 m wide passage through it
    'narrow_passage': dict(bounds=ROOM, blocks=[(-2.5, (-5 + -2.5) / 2, 1.6, 2.5), (-2.5, (-1.5 + 1) / 2, 1.6, 2.5)],
                           cyls=[], goal=B),
    # a 0.2 m wall with a 1.0 m door near its north end, goal straight east of A behind the wall
    'doorway':        dict(bounds=ROOM, blocks=wall_x(-2.5, -5.0, 1.0, door=-0.8), cyls=[], goal=(-0.5, -3.5, 0.0)),
    # L-shaped corridor 1.5 m wide: east along y = -3.5, then north
    'corner':         dict(bounds=(-6.0, 0.0, -4.25, 2.0), blocks=[(-3.75, -0.375, 4.5, 4.75)], cyls=[],
                           goal=(-0.75, 1.0, round(math.pi / 2, 4))),
    # three rooms: west (start) -> door at y = -1.5 -> south-east -> door at x = 1.0 -> north-east (goal)
    'rooms':          dict(bounds=(-6.0, 2.0, -5.0, 3.0), blocks=wall_x(-2.0, -5.0, 3.0, door=-1.5) + wall_y(0.0, -2.0, 2.0, door=1.0),
                           cyls=[], goal=(0.0, 2.0, round(math.pi, 4)), cam=True),
}


def obstacles(w):
    """All occupied rectangles (x, y, sx, sy) and circles (x, y, r) of a world."""
    return walls(*w['bounds']) + w['blocks'], w['cyls']


def rect_dist(px, py, r):
    x, y, sx, sy = r
    return math.hypot(max(abs(px - x) - sx / 2, 0.0), max(abs(py - y) - sy / 2, 0.0))


def clearance(w, px, py):
    """Distance from a point to the nearest obstacle surface (0 inside one)."""
    rects, circles = obstacles(w)
    return min([rect_dist(px, py, r) for r in rects] + [max(math.hypot(px - x, py - y) - r, 0.0) for x, y, r in circles])


def world_sdf(name, w):
    up = (PKG.parent / 'third_party/unitree-go2-ros2/robots/configs/go2_config/worlds/default.world').read_text()
    g0 = up.index("<model name='ground_plane'>"); g1 = up.index('</model>', g0) + len('</model>')   # CHAMP's high-friction ground
    rects, circles = obstacles(w)
    m = [up[g0:g1].replace('Gazebo/Grey', 'Gazebo/DarkGrey')]
    m += [box_model(f'wall{i}', *r, WALL_H, 'White') for i, r in enumerate(rects[:4])]
    m += [box_model(f'block{i}', *r, OBS_H, 'WoodPallet' if i % 2 else 'Bricks') for i, r in enumerate(rects[4:])]
    m += [cyl_model(f'pillar{i}', *c, OBS_H, 'Blue' if i % 2 else 'Orange') for i, c in enumerate(circles)]
    m += [marker('marker_A', *A, '0.1 0.8 0.2'), marker('marker_B', *w['goal'][:2], '0.9 0.15 0.1')]
    if w.get('cam'):                              # fixed oblique view from the south, for the videos
        x0, x1, y0, y1 = w['bounds']
        tx, ty = (x0 + x1) / 2, (y0 + y1) / 2 - 0.5
        cx, cy, cz = tx, y0 - 5.0, 8.0
        yaw = math.atan2(ty - cy, tx - cx); pitch = math.atan2(cz, math.hypot(tx - cx, ty - cy))
        m.append(f'''    <model name="spectator"><static>1</static><pose>{cx} {cy} {cz} 0 {pitch:.3f} {yaw:.3f}</pose><link name="l">
      {camera("spectator", "0 0 0 0 0 0", fov=1.1)}</link></model>''')
    return f'''<?xml version="1.0"?>
<sdf version="1.6"><world name="{name}">
    <light name="sun" type="directional"><cast_shadows>1</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular><direction>-0.5 0.5 -1</direction></light>
    <gravity>0 0 -9.8</gravity>
    <physics name="default_physics" default="0" type="ode"><max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor><real_time_update_rate>1000</real_time_update_rate></physics>
    <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.7 0.78 0.9 1</background><shadows>1</shadows></scene>
{chr(10).join(m)}
</world></sdf>
'''


def occupancy(w):
    """Boolean grid (row 0 = min y) and its origin; a cell is occupied when its centre lies inside an obstacle."""
    x0, x1, y0, y1 = w['bounds']
    ox, oy = x0 - MARGIN, y0 - MARGIN
    W, H = int(round((x1 - x0 + 2 * MARGIN) / RES)), int(round((y1 - y0 + 2 * MARGIN) / RES))
    X, Y = np.meshgrid(ox + (np.arange(W) + .5) * RES, oy + (np.arange(H) + .5) * RES)
    occ = np.zeros((H, W), bool)
    rects, circles = obstacles(w)
    for x, y, sx, sy in rects:
        occ |= (abs(X - x) <= sx / 2) & (abs(Y - y) <= sy / 2)
    for x, y, r in circles:
        occ |= (X - x) ** 2 + (Y - y) ** 2 <= r ** 2
    return occ, (ox, oy)


def main():
    for name, w in WORLDS.items():
        (HERE / f'{name}.world').write_text(world_sdf(name, w))
        if name == 'obstacles_mod':
            continue                              # deliberately no map: 10.15 navigates it on obstacles.yaml
        occ, (ox, oy) = occupancy(w)
        img = np.where(occ, 0, 254).astype(np.uint8)[::-1]          # PGM row 0 = top = max y
        (HERE / f'{name}.pgm').write_bytes(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode() + img.tobytes())
        (HERE / f'{name}.yaml').write_text(f'image: {name}.pgm\nmode: trinary\nresolution: {RES}\norigin: [{ox}, {oy}, 0.0]\n'
                                           'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        print(f'{name:15s} {img.shape[1]}x{img.shape[0]} cells, goal {w["goal"]}')


if __name__ == '__main__':
    main()
