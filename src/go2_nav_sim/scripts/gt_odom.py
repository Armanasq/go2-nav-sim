#!/usr/bin/env python3
"""Simulator ground-truth -> /odom + TF odom->base_footprint (planar). Replaces CHAMP's leg odometry in Gazebo,
where it is unusable (no foot-contact sensing: measured 0.55 m reported vs 3.2 m travelled)."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GtOdom(Node):
    def __init__(self):
        super().__init__('gt_odom')
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Odometry, 'odom_gt', self.cb, 50)

    def cb(self, m):
        yaw = yaw_of(m.pose.pose.orientation)
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = m.header.stamp, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = m.pose.pose.position.x, m.pose.pose.position.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        vx, vy = m.twist.twist.linear.x, m.twist.twist.linear.y            # p3d reports world-frame velocity
        o.twist.twist.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy   # -> body frame
        o.twist.twist.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        o.twist.twist.angular.z = m.twist.twist.angular.z
        self.pub.publish(o)
        t = TransformStamped()
        t.header, t.child_frame_id = o.header, 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = o.pose.pose.position.x, o.pose.pose.position.y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(GtOdom())


if __name__ == '__main__':
    main()
