#!/usr/bin/env python3
"""
Republishes nav_msgs/Odometry (from the bridged /odom topic) as a
plain TF transform: odom -> base_footprint (no model-name prefix).

This exists because gz-sim's DiffDrive system auto-prefixes TF frame
names with the model's scoped name (e.g. "diff_bot/odom") when
publishing on /model/<name>/tf, regardless of the odom_frame/base_frame
values set in SDF. Nav2 expects plain frame names, so we bridge the
odometry data (not the TF) and broadcast it ourselves under the right
names.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 50)

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()