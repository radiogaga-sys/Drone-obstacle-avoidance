#!/usr/bin/env python3
"""
Multi-waypoint mission control for ego-planner (MANUAL_TARGET mode).

Based on mission_control.py. Differences:
  - Waypoints loaded from YAML instead of CLI args
  - After reaching each waypoint, automatically advances to next
  - Subscribes to /Odometry (camera_init frame) for position

Usage:
  ros2 run ego_planner mission_control_multi.py
  ros2 run ego_planner mission_control_multi.py --ros-args -p waypoints_yaml:=/path/to/yaml
"""

import os
import sys
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from quadrotor_msgs.msg import PositionCommand


class MissionControlMulti(Node):
    def __init__(self, waypoints_yaml: str):
        super().__init__("mission_control_multi")

        # ── Load waypoints from YAML ──────────────────────────────
        with open(waypoints_yaml, 'r') as f:
            config = yaml.safe_load(f)

        self._waypoints = []
        for wp in config['waypoints']:
            self._waypoints.append({
                'x': float(wp[0]), 'y': float(wp[1]), 'z': float(wp[2]),
            })

        defaults = config.get('defaults', {})
        self._takeoff_z = defaults.get('takeoff_height', 1.5)
        self._complete_dist = defaults.get('complete_dist', 0.5)
        self._total_wp = len(self._waypoints)

        self.get_logger().info(f"Loaded {self._total_wp} waypoints from {waypoints_yaml}")
        for i, wp in enumerate(self._waypoints):
            self.get_logger().info(f"  wp{i}: ({wp['x']:.1f}, {wp['y']:.1f}, {wp['z']:.1f})")
        self.get_logger().info(f"takeoff={self._takeoff_z}m  complete_dist={self._complete_dist}m")

        # ── State (mirrors mission_control.py) ──────────────────
        self._odom = None
        self._connected = False
        self._armed = False
        self._mode = ""
        self._takeoff_reached = False
        self._current_wp = 0
        self._goal_sent = False
        self._cmd_bridge_active = False
        self._handover_counter = 0
        self._mission_complete = False

        # Per-phase retry counters
        self._mode_attempts = 0
        self._arm_attempts = 0
        self._hover_count = 0
        self._max_retries = 50

        # QoS: MAVROS publishes state + odom as BEST_EFFORT
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriptions
        self._state_sub = self.create_subscription(
            State, "/mavros/state", self._state_cb, qos)
        self._odom_sub = self.create_subscription(
            Odometry, "/mavros/local_position/odom", self._odom_cb, qos)
        self._cmd_sub = self.create_subscription(
            PositionCommand, "/position_cmd", self._cmd_cb, 10)

        # Publishers
        self._setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self._goal_pub = self.create_publisher(
            PoseStamped, "/goal_point", 10)

        # Service clients
        self._set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self._arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")

        # Control loop at 10 Hz
        self._timer = self.create_timer(0.1, self._control_loop)

    # ── Callbacks ──────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    def _state_cb(self, msg: State):
        self._connected = msg.connected
        self._armed = msg.armed
        self._mode = msg.mode

    def _cmd_cb(self, msg: PositionCommand):
        self._cmd_bridge_active = True

    # ── Helpers ────────────────────────────────────────────────────
    def _send_setpoint(self, x: float, y: float, z: float):
        pt = PoseStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.pose.position.x = float(x)
        pt.pose.position.y = float(y)
        pt.pose.position.z = float(z)
        pt.pose.orientation.w = 1.0
        self._setpoint_pub.publish(pt)

    def _publish_goal(self):
        wp = self._waypoints[self._current_wp]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "camera_init"
        goal.pose.position.x = wp['x']
        goal.pose.position.y = wp['y']
        goal.pose.position.z = wp['z']
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)
        self.get_logger().info(
            f"Goal wp{self._current_wp} sent: "
            f"({wp['x']:.1f}, {wp['y']:.1f}, {wp['z']:.1f})"
        )

    def _dist_to_current_wp(self):
        if self._odom is None:
            return float('inf')
        wp = self._waypoints[self._current_wp]
        dx = self._odom.pose.pose.position.x - wp['x']
        dy = self._odom.pose.pose.position.y - wp['y']
        dz = self._odom.pose.pose.position.z - wp['z']
        return (dx*dx + dy*dy + dz*dz) ** 0.5

    # ── Main FSM ───────────────────────────────────────────────────
    def _control_loop(self):
        """Mirrors mission_control.py structure with multi-waypoint support."""
        if not self._connected or self._odom is None:
            self._send_setpoint(0.0, 0.0, 0.0)
            return

        cur_x = self._odom.pose.pose.position.x
        cur_y = self._odom.pose.pose.position.y
        cur_z = self._odom.pose.pose.position.z

        # ── Phase 1: OFFBOARD ──────────────────────────────────────
        if not self._armed and self._mode != "OFFBOARD":
            self._send_setpoint(0.0, 0.0, 0.0)
            if self._mode_attempts < self._max_retries:
                if self._mode_attempts == 0:
                    self.get_logger().info(f"Phase 1: {self._mode} → OFFBOARD...")
                self._set_mode_cli.call_async(
                    SetMode.Request(custom_mode="OFFBOARD"))
                self._mode_attempts += 1
            return

        if self._mode == "OFFBOARD":
            self._mode_attempts = 0

        # ── Phase 2: Arm ───────────────────────────────────────────
        if self._mode == "OFFBOARD" and not self._armed:
            self._send_setpoint(0.0, 0.0, 0.0)
            if self._arm_attempts < self._max_retries:
                if self._arm_attempts == 0:
                    self.get_logger().info("Phase 2: Arming...")
                self._arm_cli.call_async(CommandBool.Request(value=True))
                self._arm_attempts += 1
            return

        # ── Phase 3: Takeoff ───────────────────────────────────────
        if (self._armed and self._mode == "OFFBOARD"
                and not self._takeoff_reached):
            self._send_setpoint(0.0, 0.0, self._takeoff_z)
            if abs(cur_z - self._takeoff_z) < 0.3:
                self._hover_count += 1
                if self._hover_count > 30:
                    self.get_logger().info(
                        f"Takeoff complete at z={cur_z:.2f}m")
                    self._takeoff_reached = True
            return

        # ── Phase 4: Send goal to ego-planner ──────────────────────
        if self._takeoff_reached and not self._goal_sent:
            self._publish_goal()
            self._goal_sent = True
            self._cmd_bridge_active = False
            self._handover_counter = 0

        # ── Phase 5: Transition to cmd_bridge ──────────────────────
        if self._goal_sent and not self._cmd_bridge_active:
            self._send_setpoint(cur_x, cur_y, self._takeoff_z)
            return

        if self._goal_sent and self._cmd_bridge_active:
            self._handover_counter += 1
            if self._handover_counter <= 20:
                self._send_setpoint(cur_x, cur_y, self._takeoff_z)
                return
            elif self._handover_counter == 21:
                self.get_logger().info("Handover complete — cmd_bridge in control")

        # ── Phase 6: Monitor arrival ───────────────────────────────
        if self._mission_complete:
            return

        if self._goal_sent and self._cmd_bridge_active:
            dist = self._dist_to_current_wp()
            if dist < self._complete_dist:
                self.get_logger().info(
                    f"✓ Waypoint {self._current_wp} reached "
                    f"(dist={dist:.2f}m)"
                )
                self._current_wp += 1
                if self._current_wp < self._total_wp:
                    self.get_logger().info(
                        f"Advancing to wp{self._current_wp} "
                        f"({self._waypoints[self._current_wp]['x']:.1f}, "
                        f"{self._waypoints[self._current_wp]['y']:.1f}, "
                        f"{self._waypoints[self._current_wp]['z']:.1f})"
                    )
                    self._goal_sent = False
                else:
                    self._report_completion()

    def _report_completion(self):
        self._mission_complete = True
        self.get_logger().info("=" * 50)
        self.get_logger().info(f" MISSION COMPLETE — {self._total_wp}/{self._total_wp} waypoints reached")
        self.get_logger().info("=" * 50)


def main():
    rclpy.init(args=sys.argv)

    node_for_params = rclpy.create_node('_mission_control_multi_params')
    node_for_params.declare_parameter('waypoints_yaml', '')

    _script_dir = os.path.dirname(os.path.realpath(__file__))
    _ws_yaml = os.path.abspath(os.path.join(
        _script_dir, '..', '..', '..', '..', 'config', 'test_5_waypoints.yaml'))

    yaml_path = node_for_params.get_parameter('waypoints_yaml').value
    if not yaml_path or not os.path.exists(yaml_path):
        if os.path.exists(_ws_yaml):
            yaml_path = _ws_yaml
        else:
            node_for_params.get_logger().fatal("waypoints_yaml not found")
            node_for_params.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    node_for_params.destroy_node()

    node = MissionControlMulti(yaml_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
