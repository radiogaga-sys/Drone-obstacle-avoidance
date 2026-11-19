#!/usr/bin/env python3
"""
Real-drone TF tree: one node, three static transforms.

  - body       -> base_link     static    SLAM IMU center to drone body center
  - base_link  -> mid360        static    LiDAR mounting offset
  - base_link  -> camera_down   static    Downward-facing USB camera

Resulting tree (ONE connected tree):

  camera_init ──► body ──► base_link ──► livox_frame
                                     └──► camera_down ──► tag36h11:<id>
  ^^^^^^^^^^^^ published by FAST_LIO       ^^^^^^^^^^^^ published by apriltag_ros

This node deliberately does NOT publish odom -> base_link.
FAST_LIO already publishes camera_init -> body, and tf2 allows only ONE parent
per frame: it keeps whichever edge was written last and silently drops the
other (a static edge is not privileged). The MAVROS-driven odom -> base_link
edge used to be republished at 10-50 Hz while body -> base_link was sent once
at startup, so base_link ended up parented to odom, the body subtree became a
second, unconnected tree, and every body -> camera_down -> tag lookup failed:

    ConnectivityException: ... Tf has two or more unconnected trees

which made tag_pose_relay unable to ever publish /tag_detection/pose.
The PX4 estimate is still available on the /mavros/local_position/odom TOPIC;
nothing in this workspace needs it as a TF frame.

Usage:
  python3 tf_broadcaster.py
  python3 tf_broadcaster.py --ros-args \
      -p body_frame:=body \
      -p base_link_frame:=base_link \
      -p body_x:=0.0 -p body_y:=0.0 -p body_z:=0.0 \
      -p body_roll:=0.0 -p body_pitch:=0.0 -p body_yaw:=0.0 \
      -p mid360_x:=0.0 -p mid360_y:=0.0 -p mid360_z:=0.1 \
      -p mid360_roll:=0.0 -p mid360_pitch:=0.0 -p mid360_yaw:=0.0 \
      -p camera_down_x:=0.0 -p camera_down_y:=0.0 -p camera_down_z:=-0.15 \
      -p camera_down_roll:=0.0 -p camera_down_pitch:=1.57 -p camera_down_yaw:=0.0

Verify (must print numbers, not "two or more unconnected trees"):
  ros2 run tf2_ros tf2_echo body camera_down
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class TfBroadcaster(Node):
    def __init__(self):
        super().__init__("tf_broadcaster")

        # --- Frame names ------------------------------------------
        self.declare_parameter("body_frame", "body")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("mid360_frame", "livox_frame")
        self.declare_parameter("camera_down_frame", "camera_down")
        self._body_frame = self.get_parameter("body_frame").value
        self._base_frame = self.get_parameter("base_link_frame").value
        self._mid360_frame = self.get_parameter("mid360_frame").value
        self._camera_down_frame = self.get_parameter("camera_down_frame").value

        # --- body -> base_link static offset ----------------------
        self.declare_parameter("body_x", 0.0)
        self.declare_parameter("body_y", 0.0)
        self.declare_parameter("body_z", 0.0)
        self.declare_parameter("body_roll", 0.0)
        self.declare_parameter("body_pitch", 0.0)
        self.declare_parameter("body_yaw", 0.0)

        # --- base_link -> mid360 static offset --------------------
        self.declare_parameter("mid360_x", 0.0)
        self.declare_parameter("mid360_y", 0.0)
        self.declare_parameter("mid360_z", 0.1)
        self.declare_parameter("mid360_roll", 0.0)
        self.declare_parameter("mid360_pitch", 0.0)
        self.declare_parameter("mid360_yaw", 0.0)

        # --- base_link -> camera_down static offset ---------------
        self.declare_parameter("camera_down_x", 0.0)
        self.declare_parameter("camera_down_y", 0.0)
        self.declare_parameter("camera_down_z", -0.15)
        self.declare_parameter("camera_down_roll", 0.0)
        self.declare_parameter("camera_down_pitch", 1.57)  # ~90 deg downward
        self.declare_parameter("camera_down_yaw", 0.0)

        # --- Static TF broadcaster --------------------------------
        # All three edges are static: published once on /tf_static with
        # transient_local durability, so late subscribers still get them.
        self._static_br = StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        self.get_logger().info(
            f"TF tree (static, /tf_static):\n"
            f"  {self._body_frame} -> {self._base_frame}\n"
            f"  {self._base_frame} -> {self._mid360_frame}\n"
            f"  {self._base_frame} -> {self._camera_down_frame}\n"
            f"  (no odom -> base_link: {self._body_frame} already has "
            f"camera_init as parent, published by FAST_LIO)"
        )

    def _quaternion_from_euler(self, roll: float, pitch: float, yaw: float):
        """Convert Euler angles to quaternion (avoids tf_transformations dependency)."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy,   # z
            cr * cp * cy + sr * sp * sy,   # w
        )

    def _make_static_tf(self, parent: str, child: str, x: float, y: float,
                        z: float, roll: float, pitch: float,
                        yaw: float) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = self._quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t

    def _publish_static_transforms(self):
        transforms = []

        # body -> base_link  (IMU center -> drone body center)
        transforms.append(self._make_static_tf(
            self._body_frame, self._base_frame,
            self.get_parameter("body_x").value,
            self.get_parameter("body_y").value,
            self.get_parameter("body_z").value,
            self.get_parameter("body_roll").value,
            self.get_parameter("body_pitch").value,
            self.get_parameter("body_yaw").value,
        ))

        # base_link -> mid360  (drone body -> LiDAR sensor)
        transforms.append(self._make_static_tf(
            self._base_frame, self._mid360_frame,
            self.get_parameter("mid360_x").value,
            self.get_parameter("mid360_y").value,
            self.get_parameter("mid360_z").value,
            self.get_parameter("mid360_roll").value,
            self.get_parameter("mid360_pitch").value,
            self.get_parameter("mid360_yaw").value,
        ))

        # base_link -> camera_down  (drone body -> downward USB camera)
        transforms.append(self._make_static_tf(
            self._base_frame, self._camera_down_frame,
            self.get_parameter("camera_down_x").value,
            self.get_parameter("camera_down_y").value,
            self.get_parameter("camera_down_z").value,
            self.get_parameter("camera_down_roll").value,
            self.get_parameter("camera_down_pitch").value,
            self.get_parameter("camera_down_yaw").value,
        ))

        self._static_br.sendTransform(transforms)


def main():
    rclpy.init()
    node = TfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
