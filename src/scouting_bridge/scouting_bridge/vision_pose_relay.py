#!/usr/bin/env python3
"""
Relay FAST_LIO_ROS2 /Odometry to /mavros/vision_pose/pose for PX4 EKF2 fusion.

FAST_LIO_ROS2 publishes: /Odometry (frame_id: camera_init, child: body)
PX4 EKF2 expects:        /mavros/vision_pose/pose (PoseStamped, frame: camera_init)

On real hardware, PX4 has no Gazebo ground truth. It must receive external
vision odometry via this topic for EKF2 fusion (EKF2_AID_MASK bit 1, EKF2_EV_CTRL=7).

This relay mirrors what FAST-LIVO2 does natively in LIVMapper::publish_mavros().

Usage:
  python3 vision_pose_relay.py
  python3 vision_pose_relay.py --ros-args -p odom_topic:=/Odometry
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class VisionPoseRelay(Node):
    def __init__(self):
        super().__init__("vision_pose_relay")

        self.declare_parameter("odom_topic", "/Odometry")
        odom_topic = self.get_parameter("odom_topic").value

        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10)

        self._vision_pub = self.create_publisher(
            PoseStamped, "/mavros/vision_pose/pose", 10)

        self.get_logger().info(
            f"Relaying {odom_topic} -> /mavros/vision_pose/pose "
            f"(frame_id: camera_init)"
        )

    def _odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = "camera_init"
        pose.pose = msg.pose.pose
        self._vision_pub.publish(pose)


def main():
    rclpy.init()
    node = VisionPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
