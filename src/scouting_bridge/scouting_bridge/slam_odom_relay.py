#!/usr/bin/env python3
"""
Bridge SLAM odometry (FAST-LIVO2 or FAST_LIO_ROS2) to ego-planner's
expected /odom_world topic.

FAST-LIVO2 publishes:  /aft_mapped_to_init (frame: camera_init, child: aft_mapped)
FAST_LIO_ROS2 publishes: /Odometry          (frame: camera_init, child: body)

Ego-planner subscribes to odom_world (frame: world)

This relay:
  1. Republishes odometry with frame_id="world", preserves original timestamp
  2. Also publishes on grid_map/odom (used by plan_env if not remapped)

NOTE: Does NOT publish TF (world -> base_link) to avoid conflict with
MAVROS which also publishes map -> base_link. Ego-planner reads pose
from the Odometry message directly, not from TF.

Usage:
  # For FAST-LIVO2:
  python3 slam_odom_relay.py odom_world /aft_mapped_to_init
  # For FAST_LIO_ROS2 (default):
  python3 slam_odom_relay.py odom_world /Odometry
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class SlamOdomRelay(Node):
    def __init__(self):
        super().__init__("slam_odom_relay")

        self.declare_parameter("odom_topic_in", "/Odometry")
        self.declare_parameter("odom_topic_out", "odom_world")
        odom_topic_in = self.get_parameter("odom_topic_in").value
        odom_topic_out = self.get_parameter("odom_topic_out").value

        # SLAM output (LiDAR odometry in camera_init frame)
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic_in, self._odom_callback, 10
        )

        # Ego-planner expected input (odometry in world frame)
        self._odom_pub = self.create_publisher(Odometry, odom_topic_out, 10)

        # grid_map/odom — plan_env uses this to localize the occupancy grid.
        # The launch file remaps grid_map/odom -> odom_world, so this is a
        # safety fallback; won't hurt if no one subscribes.
        self._grid_odom_pub = self.create_publisher(Odometry, "grid_map/odom", 10)

        self.get_logger().info(
            f"Relaying {odom_topic_in} -> {odom_topic_out} "
            f"(frame_id: camera_init -> world)"
        )

    def _build_odom(self, msg: Odometry) -> Odometry:
        """Convert SLAM odometry to world-frame odometry for ego-planner."""
        odom = Odometry()
        # Preserve original timestamp for proper TF/sensor sync
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"

        # Camera_init == world origin (identity transform at startup)
        odom.pose.pose = msg.pose.pose
        odom.twist.twist = msg.twist.twist
        return odom

    def _odom_callback(self, msg: Odometry):
        odom = self._build_odom(msg)

        # Primary output: ego-planner's odom_world
        self._odom_pub.publish(odom)

        # Secondary: grid_map/odom for occupancy mapping
        self._grid_odom_pub.publish(odom)


def main():
    rclpy.init()
    node = SlamOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
