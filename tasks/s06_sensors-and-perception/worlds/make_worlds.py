#!/usr/bin/env python3
"""Write the two Gazebo inputs of stage 6 from the constants below (single source of truth for their true size).
  platform.world  the arena (src/go2_nav_sim/worlds/arena.world) plus one static platform box in front of marker A, for 6.16/6.17
  new_box.sdf     a static box that 6.14 spawns into the running arena while the robot walks
usage: python3 tasks/s06_sensors-and-perception/worlds/make_worlds.py"""
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
ARENA = HERE.parents[2] / 'src/go2_nav_sim/worlds/arena.world'
# platform: centre x, y, size x, size y, height (world frame, metres). Near edge x = -2.7, 1.52 m ahead of the camera at A.
PLATFORM = (-2.2, -3.5, 1.0, 1.2, 0.12)
# new obstacle for 6.14: centre x, y, size x, size y, height. West face x = 1.2, 0.7 m before the pallet face (x = 1.9).
NEW_BOX = (1.4, -2.5, 0.4, 0.6, 0.5)


def box(name, x, y, sx, sy, h, material):
    return (f'<model name="{name}"><static>1</static><pose>{x} {y} {h / 2} 0 0 0</pose><link name="l">'
            f'<collision name="c"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>'
            f'<visual name="v"><geometry><box><size>{sx} {sy} {h}</size></box></geometry><material><script>'
            f'<uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/{material}</name></script></material>'
            f'</visual></link></model>')


if __name__ == '__main__':
    w = ARENA.read_text()
    (HERE / 'platform.world').write_text(w.replace('</world>', '    ' + box('platform', *PLATFORM, 'Wood') + '\n</world>'))
    x, y, sx, sy, h = NEW_BOX                                 # spawn_entity places the model origin; the pose lifts it by h/2
    (HERE / 'new_box.sdf').write_text('<?xml version="1.0"?>\n<sdf version="1.6">'
                                      + box('new_box', 0, 0, sx, sy, h, 'Red') + '</sdf>\n')
    print('wrote', HERE / 'platform.world', HERE / 'new_box.sdf')
