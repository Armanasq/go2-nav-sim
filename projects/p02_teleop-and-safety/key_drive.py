#!/usr/bin/env python3
"""Drive teleop_twist_keyboard from a script, as if a person held keys down, so a teleop run can be repeated.
usage: python3 projects/p02_teleop-and-safety/key_drive.py "j 6.3, i 8, - 3"
Each item is a key and how many simulated seconds to hold it; '-' means release every key (send nothing).
teleop_twist_keyboard runs on a pseudo-terminal and publishes to teleop/cmd_vel, the input of safety_node.py."""
import os, pty, subprocess, sys, threading, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

REPEAT = 1 / 30          # a held key repeats about 30 times a second (xset q: repeat rate 33)


def main():
    plan = [(k, float(s)) for k, s in (item.split() for item in sys.argv[1].split(','))]
    master, slave = pty.openpty()
    teleop = subprocess.Popen(['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                               '--ros-args', '-r', 'cmd_vel:=teleop/cmd_vel'],
                              stdin=slave, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    clock = Node('key_drive', parameter_overrides=[Parameter('use_sim_time', value=True)])
    now = lambda: clock.get_clock().now().nanoseconds * 1e-9
    spinner = threading.Thread(target=rclpy.spin, args=(clock,))   # keeps the simulated clock current
    spinner.start()
    while now() == 0.0:
        time.sleep(0.1)                              # wait for /clock
    time.sleep(3)                                    # let teleop start and connect
    try:
        for key, secs in plan:
            print(f'{now():8.2f}  {key} for {secs} s', flush=True)
            t_end = now() + secs
            while now() < t_end:
                if key != '-':
                    os.write(master, key.encode())
                time.sleep(REPEAT)
    finally:
        os.write(master, b'\x03')                    # Ctrl-C: teleop sends a zero twist and exits
        teleop.wait(timeout=10)
        rclpy.try_shutdown(); spinner.join()        # stop the spin thread before exit, or the C++ side aborts


if __name__ == '__main__':
    main()
