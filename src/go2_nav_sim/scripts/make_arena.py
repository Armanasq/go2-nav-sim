#!/usr/bin/env python3
"""Generate worlds/arena.world and maps/arena.{pgm,yaml} from ONE obstacle list (single source of truth).
Run from the package root:  python3 scripts/make_arena.py"""
import math, pathlib
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
HALF_X, HALF_Y, WALL_T, WALL_H = 6.0, 5.0, 0.2, 1.2          # arena [-6,6]x[-5,5] m
A, B = (-4.5, -3.5), (4.0, 3.0)                                  # start / goal (world frame)
BOXES = [  # x, y, size_x, size_y, height, gazebo material
    (-1.5, -1.5, 1.5, 0.8, 0.5, "WoodPallet"), (1.0, 0.0, 0.8, 2.4, 1.0, "Bricks"),
    (2.5, -2.5, 1.2, 1.2, 0.45, "WoodPallet"), (-2.5, 2.0, 2.0, 0.6, 0.8, "Bricks")]
CYLS = [  # x, y, radius, height, material
    (0.0, 3.4, 0.45, 1.5, "Blue"), (-3.5, 0.0, 0.40, 0.7, "Orange")]
WALLS = [(0, HALF_Y, 2*HALF_X + WALL_T, WALL_T), (0, -HALF_Y, 2*HALF_X + WALL_T, WALL_T),
         (HALF_X, 0, WALL_T, 2*HALF_Y), (-HALF_X, 0, WALL_T, 2*HALF_Y)]
RES, MARGIN = 0.05, 0.5

def mat(name):
    return f'<material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/{name}</name></script></material>'

def floor_grid(step=1.0):
    # 1 m grid of thin visual-only lines: gives the RGB camera perspective/motion cues and a scale reference
    lines = []
    for i, x in enumerate(np.arange(-HALF_X, HALF_X + 1e-6, step)):
        lines.append(f'<visual name="gx{i}"><pose>{x:.2f} 0 0.001 0 0 0</pose><geometry><box><size>0.03 {2*HALF_Y} 0.002</size></box></geometry>{mat("White")}</visual>')
    for i, y in enumerate(np.arange(-HALF_Y, HALF_Y + 1e-6, step)):
        lines.append(f'<visual name="gy{i}"><pose>0 {y:.2f} 0.001 0 0 0</pose><geometry><box><size>{2*HALF_X} 0.03 0.002</size></box></geometry>{mat("White")}</visual>')
    return '    <model name="floor_grid"><static>1</static><link name="l">' + ''.join(lines) + '</link></model>'

def box_model(name, x, y, sx, sy, h, material):
    return f'''    <model name="{name}"><static>1</static><pose>{x} {y} {h/2} 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{sx} {sy} {h}</size></box></geometry>
      {mat(material)}</visual></link></model>'''

def cyl_model(name, x, y, r, h, material):
    return f'''    <model name="{name}"><static>1</static><pose>{x} {y} {h/2} 0 0 0</pose><link name="l">
      <collision name="c"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>
      <visual name="v"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
      {mat(material)}</visual></link></model>'''

def marker(name, x, y, rgb):  # visual-only disc on the floor
    return f'''    <model name="{name}"><static>1</static><pose>{x} {y} 0.01 0 0 0</pose><link name="l">
      <visual name="v"><geometry><cylinder><radius>0.4</radius><length>0.02</length></cylinder></geometry>
      <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse><emissive>{rgb} 1</emissive></material></visual></link></model>'''

def camera(name, pose, w=960, h=540, fov=1.2, rate=10):
    return f'''<sensor name="{name}_sensor" type="camera"><always_on>1</always_on><update_rate>{rate}</update_rate>
        <camera><horizontal_fov>{fov}</horizontal_fov><image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
        <clip><near>0.1</near><far>40</far></clip></camera>
        <plugin name="{name}_plugin" filename="libgazebo_ros_camera.so"><ros><namespace>{name}</namespace></ros>
        <camera_name>cam</camera_name><frame_name>{name}_optical</frame_name></plugin><pose>{pose}</pose></sensor>'''

def world_sdf():
    ground = (ROOT.parent / "third_party/unitree-go2-ros2/robots/configs/go2_config/worlds/default.world").read_text()
    # reuse the upstream ground plane (high-friction, tuned for CHAMP) verbatim
    g0 = ground.index("<model name='ground_plane'>"); g1 = ground.index("</model>", g0) + len("</model>")
    m = [ground[g0:g1].replace("Gazebo/Grey", "Gazebo/DarkGrey"), floor_grid()]
    m += [box_model(f"wall{i}", *w, WALL_H, "White") for i, w in enumerate(WALLS)]
    m += [box_model(f"box{i}", *b[:4], b[4], b[5]) for i, b in enumerate(BOXES)]
    m += [cyl_model(f"pillar{i}", *c) for i, c in enumerate(CYLS)]
    m += [marker("marker_A", *A, "0.1 0.8 0.2"), marker("marker_B", *B, "0.9 0.15 0.1")]
    # fixed spectator camera outside the south wall, oblique view of the whole arena (for the demo video)
    cx, cy, cz, tx, ty = 0.0, -11.0, 9.5, 0.0, -0.5
    yaw = math.atan2(ty - cy, tx - cx); pitch = math.atan2(cz, math.hypot(tx - cx, ty - cy))
    m.append(f'''    <model name="spectator"><static>1</static><pose>{cx} {cy} {cz} 0 {pitch:.3f} {yaw:.3f}</pose><link name="l">
      {camera("spectator", "0 0 0 0 0 0", fov=1.3)}</link></model>''')
    return f'''<?xml version="1.0"?>
<sdf version="1.6"><world name="arena">
    <light name="sun" type="directional"><cast_shadows>1</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular><direction>-0.5 0.5 -1</direction></light>
    <gravity>0 0 -9.8</gravity>
    <physics name="default_physics" default="0" type="ode"><max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor><real_time_update_rate>1000</real_time_update_rate></physics>
    <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.7 0.78 0.9 1</background><shadows>1</shadows></scene>
{chr(10).join(m)}
</world></sdf>
'''

def occupancy():
    x0, y0 = -HALF_X - MARGIN, -HALF_Y - MARGIN
    W, H = int((2*HALF_X + 2*MARGIN)/RES), int((2*HALF_Y + 2*MARGIN)/RES)
    xs = x0 + (np.arange(W) + .5)*RES; ys = y0 + (np.arange(H) + .5)*RES
    X, Y = np.meshgrid(xs, ys)
    occ = np.zeros((H, W), bool)
    for x, y, sx, sy in WALLS + [b[:4] for b in BOXES]:
        occ |= (abs(X - x) <= sx/2) & (abs(Y - y) <= sy/2)
    for x, y, r in [c[:3] for c in CYLS]:
        occ |= (X - x)**2 + (Y - y)**2 <= r**2
    img = np.where(occ, 0, 254).astype(np.uint8)[::-1]          # PGM row 0 = top = max y
    return img, (x0, y0)

if __name__ == "__main__":
    (ROOT / "worlds/arena.world").write_text(world_sdf())
    img, (x0, y0) = occupancy()
    (ROOT / "maps/arena.pgm").write_bytes(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode() + img.tobytes())
    (ROOT / "maps/arena.yaml").write_text(f"image: arena.pgm\nmode: trinary\nresolution: {RES}\norigin: [{x0}, {y0}, 0.0]\n"
                                         f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print("wrote world + map", img.shape, "A", A, "B", B)
