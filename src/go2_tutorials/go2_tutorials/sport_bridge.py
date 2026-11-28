"""/cmd_vel -> Go2 sport mode (chapter 13). Real Go2 EDU only, over unitree_ros2; untested on hardware.

Sends unitree_api/msg/Request on /api/sport/request: Move (api_id 1008, parameter {"x": vx, "y": vy, "z": vyaw}) while
commands are fresh, one StopMove (1003) after TIMEOUT s without one. IDs and the JSON keys are from unitree_ros2
example/src/src/common/ros2_sport_client.cpp at commit 668d1ec (research/go2-real-robot.md, section 1.2).
"""
import json
import math

API_MOVE, API_STOPMOVE = 1008, 1003
LIMITS = (0.3, 0.25, 0.5)  # vx, vy (m/s), vyaw (rad/s): this course's CHAMP limits; far inside the Move range (that range is S)
TIMEOUT = 0.5              # s without /cmd_vel before StopMove; the robot itself holds a Move for about 1 s (S)


def decide(cmd, age, stopped, limits=LIMITS, timeout=TIMEOUT):
    """cmd = last (vx, vy, vyaw) or None; age = s since it arrived; stopped = StopMove already sent.
    Returns ('move', clamped cmd), ('stop', None), or (None, None) when there is nothing to send."""
    if cmd is None or age > timeout or not all(math.isfinite(v) for v in cmd):
        return (None, None) if stopped else ('stop', None)
    return 'move', tuple(max(-lim, min(lim, v)) for v, lim in zip(cmd, limits))


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from unitree_api.msg import Request   # from unitree_ros2 (cyclonedds_ws); not part of this course's install

    rclpy.init()
    node = rclpy.create_node('sport_bridge')
    pub = node.create_publisher(Request, '/api/sport/request', 10)
    st = {'cmd': None, 't': 0.0, 'stopped': True}
    now = lambda: node.get_clock().now().nanoseconds * 1e-9

    def on_twist(m):
        st['cmd'], st['t'] = (m.linear.x, m.linear.y, m.angular.z), now()

    def tick():
        kind, v = decide(st['cmd'], now() - st['t'], st['stopped'])
        if kind is None:
            return
        req = Request()
        req.header.identity.api_id = API_MOVE if kind == 'move' else API_STOPMOVE
        if v:
            req.parameter = json.dumps({'x': v[0], 'y': v[1], 'z': v[2]})
        pub.publish(req)
        st['stopped'] = kind == 'stop'

    node.create_subscription(Twist, '/cmd_vel', on_twist, 10)
    node.create_timer(0.05, tick)   # 20 Hz
    rclpy.spin(node)


if __name__ == '__main__':
    main()
