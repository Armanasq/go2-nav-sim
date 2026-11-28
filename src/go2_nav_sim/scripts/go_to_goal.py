#!/usr/bin/env python3
"""Send the Go2 to a goal with Nav2 and report the outcome against simulator ground truth.
usage: go_to_goal.py [x y yaw]      world/map coordinates in metres (default: marker B, facing away from A)"""
import math, sys, time
import rclpy, rclpy.parameter
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

A, B = (-4.5, -3.5), (4.0, 3.0)                      # == make_arena.py


def pump(node, n=100):
    for _ in range(n):                                # spin_once() runs ONE callback; drain the queue instead
        rclpy.spin_once(node, timeout_sec=0.0)


def main():
    rclpy.init()
    gx, gy, gyaw = (float(v) for v in sys.argv[1:4]) if len(sys.argv) >= 4 else (*B, math.atan2(B[1] - A[1], B[0] - A[0]))
    nav = BasicNavigator()
    nav.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    gt = {}
    nav.create_subscription(Odometry, '/odom_gt', lambda m: gt.update(x=m.pose.pose.position.x + A[0], y=m.pose.pose.position.y + A[1],  # type: ignore[reportAttributeAccessIssue]  # rclpy types the callback message as an unbound TypeVar
                                                                       qz=m.pose.pose.orientation.z, qw=m.pose.pose.orientation.w), 10)  # type: ignore[reportAttributeAccessIssue]  # rclpy types the callback message as an unbound TypeVar
    while not gt:
        pump(nav, 20); time.sleep(0.05)
    # Localise AMCL from the true pose. A single message is not enough: it can be dropped by a few-millisecond TF race
    # ("Failed to transform initial pose in time"). A zero stamp makes AMCL use the latest TF; repeat until /amcl_pose agrees.
    amcl = {}
    nav.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: amcl.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y),  # type: ignore[reportAttributeAccessIssue]  # rclpy types the callback message as an unbound TypeVar
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    pub = nav.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    nav._waitForNodeToActivate('amcl')
    for attempt in range(40):
        pump(nav, 100); m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'          # stamp stays zero
        m.pose.pose.position.x, m.pose.pose.position.y = gt['x'], gt['y']
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = gt['qz'], gt['qw']
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05; m.pose.covariance[35] = 0.03
        pub.publish(m); time.sleep(0.75); pump(nav, 100)
        if amcl and math.hypot(amcl['x'] - gt['x'], amcl['y'] - gt['y']) < 0.25:
            break
    else:
        nav.error('AMCL did not converge to the true pose; navigation may fail'); 
    nav._waitForNodeToActivate('bt_navigator')
    goal = PoseStamped(); goal.header.frame_id = 'map'; goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = gx, gy
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    nav.info(f'goal -> ({gx:.2f}, {gy:.2f}, yaw {gyaw:.2f})')
    t0 = nav.get_clock().now(); nav.goToPose(goal); last = 0.0
    while not nav.isTaskComplete():
        pump(nav); time.sleep(0.1)
        fb = nav.getFeedback()
        if fb and time.time() - last > 5:
            last = time.time(); nav.info(f'distance remaining {fb.distance_remaining:.2f} m, recoveries {fb.number_of_recoveries}')
    res = nav.getResult(); dt = (nav.get_clock().now() - t0).nanoseconds / 1e9
    pump(nav, 300)
    err = math.hypot(gt['x'] - gx, gt['y'] - gy) if gt else float('nan')
    print(f'RESULT {res.name}  sim_time {dt:.1f} s  ground-truth final error {err:.2f} m  final pos ({gt.get("x", 0):.2f}, {gt.get("y", 0):.2f})')
    rclpy.shutdown()
    sys.exit(0 if res == TaskResult.SUCCEEDED else 1)


if __name__ == '__main__':
    main()
