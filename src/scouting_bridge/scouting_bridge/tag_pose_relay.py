#!/usr/bin/env python3
"""
Convert AprilTag TF detections to world-frame PoseStamped messages.

apriltag_ros publishes tag poses as TF transforms:
  camera_down → tag36h11:<id>

This node chains the full transform:
  tag_world = T_world→body * T_body→base_link * T_base_link→camera_down * T_camera_down→tag

Output:
  /tag_detection/pose   — PoseStamped (tag pose in world/camera_init frame)
  /tag_detection/id     — Int32 (detected tag ID)

Usage:
  python3 tag_pose_relay.py
  python3 tag_pose_relay.py --ros-args \
      -p camera_frame:=camera_down \
      -p tag_family:=36h11 \
      -p odom_topic:=/Odometry
"""

import re
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation


def _parse_tag_id(child_frame: str, family: str) -> int | None:
    """Extract tag ID from child_frame_id, e.g. 'tag36h11:7' → 7."""
    m = re.match(rf"tag{re.escape(family)}:(\d+)", child_frame)
    if m:
        return int(m.group(1))
    return None


class TagPoseRelay(Node):
    def __init__(self):
        super().__init__("tag_pose_relay")

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter("camera_frame", "camera_down")
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("tag_frame_prefix", "tag")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("world_frame", "camera_init")
        self.declare_parameter("body_frame", "body")
        self.declare_parameter("lost_threshold", 3)
        # Max allowed age of odom before we refuse to publish tag poses.
        # On the NUC the FAST_LIO odom stamps lag wall clock ~1s (livox lidar
        # clock offset vs system clock); the old hardcoded 1.0s threshold made
        # every publish fail ("Odometry is stale (1.0s)" forever), so the tag
        # landing controller never saw a tag even when apriltag detected it.
        self.declare_parameter("odom_max_age", 3.0)

        self._camera_frame = self.get_parameter("camera_frame").value
        self._tag_family = self.get_parameter("tag_family").value
        self._tag_prefix = self.get_parameter("tag_frame_prefix").value
        self._world_frame = self.get_parameter("world_frame").value
        self._body_frame = self.get_parameter("body_frame").value
        self._lost_threshold = self.get_parameter("lost_threshold").value
        self._odom_max_age = self.get_parameter("odom_max_age").value

        # ── State ─────────────────────────────────────────────────
        self._latest_odom: Odometry | None = None
        self._last_known_id: int | None = None  # track currently-visible tag
        self._last_known_frame: str | None = None

        # ── TF2 buffer ────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Odometry subscription (world → body) ─────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._odom_sub = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self._odom_cb, qos)

        # ── Publishers ─────────────────────────────────────────────
        self._pose_pub = self.create_publisher(PoseStamped, "/tag_detection/pose", 10)
        self._id_pub = self.create_publisher(Int32, "/tag_detection/id", 10)

        # ── Timer ──────────────────────────────────────────────────
        rate = self.get_parameter("publish_rate").value
        if rate <= 0.0:
            self.get_logger().fatal("publish_rate must be > 0")
            raise ValueError("publish_rate must be > 0")
        period = 1.0 / rate
        self._timer = self.create_timer(period, self._timer_cb)

        self._lost_counter: dict[str, int] = {}

        self.get_logger().info(
            f"TagPoseRelay ready: camera='{self._camera_frame}', "
            f"family='{self._tag_family}', odom='{self.get_parameter('odom_topic').value}'"
        )

    # ── Callbacks ──────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg

    def _timer_cb(self):
        """Periodic: find all active tag frames and publish their world poses."""
        odom = self._latest_odom
        if odom is None:
            self.get_logger().warn(
                "No odometry received yet — check /Odometry topic",
                throttle_duration_sec=5.0)
            return

        # Staleness check: skip if odom is older than odom_max_age
        odom_age = (self.get_clock().now() -
                    rclpy.time.Time.from_msg(odom.header.stamp)).nanoseconds / 1e9
        if odom_age > self._odom_max_age:
            self.get_logger().warn(
                f"Odometry is stale ({odom_age:.1f}s)", throttle_duration_sec=2.0)
            return

        # Build set of currently-active tag frames from TF
        current_tag_frames = self._find_tag_frames()
        if not current_tag_frames:
            # All tags lost: increment counter for the last known tag
            if self._last_known_id is not None:
                last_frame = self._last_known_frame
                if last_frame is not None:
                    self._lost_counter[last_frame] = (
                        self._lost_counter.get(last_frame, 0) + 1)
                    if self._lost_counter[last_frame] >= self._lost_threshold:
                        self._publish_lost()
            return

        # Collect all visible tag poses, then publish their centroid.
        # When multiple tags are on the same landing pad, the centroid
        # naturally guides the drone to the pad center.
        tag_poses: list[tuple[int, str, PoseStamped().pose]] = []

        for tag_frame in sorted(current_tag_frames):
            tag_id = _parse_tag_id(tag_frame, self._tag_family)
            if tag_id is None:
                continue

            try:
                world_pose = self._compute_world_pose(tag_frame)
            except TransformException as e:
                self.get_logger().debug(
                    f"TF lookup failed for '{tag_frame}': {e}",
                    throttle_duration_sec=2.0)
                continue

            # Reset lost counter for this tag
            self._lost_counter[tag_frame] = 0
            tag_poses.append((tag_id, tag_frame, world_pose))

        if not tag_poses:
            # All tags lost: increment counter for the last known tag
            if self._last_known_id is not None and self._last_known_frame is not None:
                self._lost_counter[self._last_known_frame] = (
                    self._lost_counter.get(self._last_known_frame, 0) + 1)
                if self._lost_counter[self._last_known_frame] >= self._lost_threshold:
                    self._publish_lost()
            return

        # Compute centroid of all visible tags
        n = len(tag_poses)
        cx = sum(p.position.x for _, _, p in tag_poses) / n
        cy = sum(p.position.y for _, _, p in tag_poses) / n
        cz = sum(p.position.z for _, _, p in tag_poses) / n

        pose_msg = PoseStamped()
        pose_msg.header.stamp = odom.header.stamp
        pose_msg.header.frame_id = self._world_frame
        pose_msg.pose.position.x = cx
        pose_msg.pose.position.y = cy
        pose_msg.pose.position.z = cz
        # Use first tag's orientation (all tags on same pad share orientation)
        first_q = tag_poses[0][2].orientation
        pose_msg.pose.orientation = first_q
        self._pose_pub.publish(pose_msg)

        # Report representative tag ID (smallest among visible)
        rep_id = min(t[0] for t in tag_poses)
        if rep_id != self._last_known_id:
            id_msg = Int32(data=rep_id)
            self._id_pub.publish(id_msg)
            self._last_known_id = rep_id
            self._last_known_frame = tag_poses[0][1]
            if n > 1:
                ids = [str(t[0]) for t in tag_poses]
                self.get_logger().info(
                    f"Tags detected: {', '.join(ids)} "
                    f"({n} tags, centroid: x={cx:.2f}, y={cy:.2f}, z={cz:.2f})"
                )
            else:
                self.get_logger().info(
                    f"Tag detected: id={rep_id}, "
                    f"x={cx:.2f}, y={cy:.2f}, z={cz:.2f}"
                )

    def _find_tag_frames(self) -> set[str]:
        """Find all currently-active tag TF frames by inspecting /tf."""
        tag_frames: set[str] = set()
        try:
            frame_names = self._tf_buffer._getFrameStrings()
        except TransformException:
            return tag_frames
        for name in frame_names:
            if name.startswith(self._tag_prefix):
                tag_frames.add(name)
        return tag_frames

    def _compute_world_pose(self, tag_frame: str):
        """
        Compute tag pose in world frame via TF chain:
        tag_in_world = T(world→body) * T(body→camera_down) * T(camera_down→tag)

        Time(0) requests the latest available transform from the TF buffer.
        """
        time_zero = rclpy.time.Time()  # epoch = "latest available"

        # Look up body → tag (chains body→base_link→camera_down→tag via TF tree)
        # Short timeout (50ms) to avoid blocking the 10Hz timer callback
        body_to_tag = self._tf_buffer.lookup_transform(
            self._body_frame, tag_frame, time_zero,
            timeout=rclpy.duration.Duration(seconds=0.05))

        # Compose: odom (world → body) * (body → tag)
        odom = self._latest_odom
        odom_p = odom.pose.pose.position
        odom_q = odom.pose.pose.orientation

        body_t = body_to_tag.transform.translation
        body_r = body_to_tag.transform.rotation

        # Rotation: odom_R_tag = odom_R_body * body_R_tag
        R_ob = Rotation.from_quat([odom_q.x, odom_q.y, odom_q.z, odom_q.w])
        R_bt = Rotation.from_quat([body_r.x, body_r.y, body_r.z, body_r.w])
        R_ot = R_ob * R_bt

        # Translation: tag_pos_in_odom = odom_pos + odom_R_body * body_pos
        t_body_in_odom = R_ob.apply([body_t.x, body_t.y, body_t.z])
        tx = odom_p.x + t_body_in_odom[0]
        ty = odom_p.y + t_body_in_odom[1]
        tz = odom_p.z + t_body_in_odom[2]

        result = PoseStamped().pose
        result.position.x = tx
        result.position.y = ty
        result.position.z = tz
        q = R_ot.as_quat()
        result.orientation.x = q[0]
        result.orientation.y = q[1]
        result.orientation.z = q[2]
        result.orientation.w = q[3]

        return result

    def _publish_lost(self):
        """Signal that the last known tag is no longer visible."""
        id_msg = Int32(data=-1)
        self._id_pub.publish(id_msg)
        if self._last_known_id is not None:
            self.get_logger().info(f"Tag {self._last_known_id} lost")
        self._last_known_id = None
        self._last_known_frame = None
        self._lost_counter.clear()


def main():
    rclpy.init()
    node = TagPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
