#!/usr/bin/env python3
"""
Mission control for ego-planner + PX4 SITL + FAST-LIVO2 integration.

Verified sequencing:
  1. Stream position setpoints at 20 Hz (setpoint_position/local, ENU)
  2. AUTO.LOITER → OFFBOARD  (skip MANUAL)
  3. Arm  (in OFFBOARD)
  4. Take off to target altitude
  5. Once hovering stably, send goal point to ego-planner
  6. Hand over setpoint control to cmd_bridge (ego-planner output)

Usage:
  ros2 run ego_planner mission_control.py [goal_x] [goal_y] [goal_z] [takeoff_z]
  python3 mission_control.py 10.0 0.0 2.0 2.0
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


class MissionControl(Node):
    def __init__(self, goal_x: float, goal_y: float, goal_z: float,
                 takeoff_z: float):
        super().__init__("mission_control")

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_z = goal_z
        self.takeoff_z = takeoff_z

        # FSM state
        self.current_odom = None
        self.connected = False
        self.armed = False
        self.mode = ""
        self.takeoff_reached = False
        self.goal_sent = False
        self.cmd_bridge_active = False
        self.transition_counter = 0

        # Per-phase retry counters
        self._mode_attempts = 0
        self._arm_attempts = 0
        self._max_retries = 100

        # QoS: MAVROS publishes state + odom as BEST_EFFORT
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self._state_sub = self.create_subscription(
            State, "/mavros/state", self._state_cb, qos
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/mavros/local_position/odom", self._odom_cb, qos
        )
        self._cmd_sub = self.create_subscription(
            PositionCommand, "/position_cmd", self._cmd_cb, 10
        )

        # Publishers
        # Use setpoint_position/local (ENU, z+ = up) — MAVROS auto-converts
        self._setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, "/goal_point", 10
        )

        # Service clients
        self._set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")
        self._arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")

        # Control loop at 20 Hz
        self._timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f"Mission: takeoff={takeoff_z}m, "
            f"goal=({goal_x:.1f}, {goal_y:.1f}, {goal_z:.1f})"
        )

    # ---- callbacks ----
    def _state_cb(self, msg: State):
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

    def _odom_cb(self, msg: Odometry):
        self.current_odom = msg

    def _cmd_cb(self, msg: PositionCommand):
        self.cmd_bridge_active = True

    # ---- helpers ----
    def _send_setpoint(self, x: float, y: float, z: float):
        """Send position setpoint in ENU (z+ = up)."""
        pt = PoseStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.pose.position.x = x
        pt.pose.position.y = y
        pt.pose.position.z = z  # ENU: positive = up
        pt.pose.orientation.w = 1.0
        self._setpoint_pub.publish(pt)

    # ---- main FSM ----
    def _control_loop(self):
        """Main FSM at 20 Hz.

        Verified sequence (matching manual CLI test):
          1. Stream setpoints continuously
          2. AUTO.LOITER → OFFBOARD  (skip MANUAL)
          3. Arm  (in OFFBOARD)
          4. Takeoff
          5. Send goal → ego-planner
          6. Hand over to cmd_bridge
          7. Monitor progress
        """
        if not self.connected or self.current_odom is None:
            # Keep streaming setpoints anyway
            self._send_setpoint(0.0, 0.0, 0.0)
            return

        cur_x = self.current_odom.pose.pose.position.x
        cur_y = self.current_odom.pose.pose.position.y
        cur_z = self.current_odom.pose.pose.position.z

        # ================================================================
        # Phase 1: AUTO.LOITER → OFFBOARD  (directly, before arming)
        # ================================================================
        if not self.armed and self.mode != "OFFBOARD":
            self._send_setpoint(0.0, 0.0, 0.0)
            if self._mode_attempts < self._max_retries:
                if self._mode_attempts == 0:
                    self.get_logger().info(
                        f"Phase 1: {self.mode} → OFFBOARD..."
                    )
                self._set_mode_cli.call_async(
                    SetMode.Request(custom_mode="OFFBOARD")
                )
                self._mode_attempts += 1
            elif self._mode_attempts == self._max_retries:
                self.get_logger().error(
                    f"OFFBOARD rejected after {self._max_retries} attempts"
                )
                self._mode_attempts += 1  # prevent overflow spam
            return

        # Reset counter once in OFFBOARD
        if self.mode == "OFFBOARD":
            self._mode_attempts = 0

        # ================================================================
        # Phase 2: Arm  (in OFFBOARD)
        # ================================================================
        if self.mode == "OFFBOARD" and not self.armed:
            self._send_setpoint(0.0, 0.0, 0.0)
            if self._arm_attempts < self._max_retries:
                if self._arm_attempts == 0:
                    self.get_logger().info("Phase 2: Arming...")
                self._arm_cli.call_async(CommandBool.Request(value=True))
                self._arm_attempts += 1
            elif self._arm_attempts == self._max_retries:
                self.get_logger().error("Arming failed")
                self._arm_attempts += 1
            return

        # ================================================================
        # Phase 3: Takeoff
        # ================================================================
        if (self.armed and self.mode == "OFFBOARD"
                and not self.takeoff_reached):
            self._send_setpoint(cur_x, cur_y, self.takeoff_z)

            if abs(cur_z - self.takeoff_z) < 0.3:
                if not hasattr(self, '_hover_count'):
                    self._hover_count = 0
                self._hover_count += 1
                if self._hover_count > 40:  # ~2 s stable at 20 Hz
                    self.get_logger().info(
                        f"Takeoff complete at z={cur_z:.2f}m"
                    )
                    self.takeoff_reached = True
            return

        # ================================================================
        # Phase 4: Send goal to ego-planner
        # ================================================================
        if self.takeoff_reached and not self.goal_sent:
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "world"
            goal.pose.position.x = self.goal_x
            goal.pose.position.y = self.goal_y
            goal.pose.position.z = self.goal_z
            goal.pose.orientation.w = 1.0
            self._goal_pub.publish(goal)
            self.get_logger().info(
                f"Goal sent: ({self.goal_x:.1f}, {self.goal_y:.1f}, "
                f"{self.goal_z:.1f})"
            )
            self.goal_sent = True

        # ================================================================
        # Phase 5: Transition to cmd_bridge
        # ================================================================
        if self.goal_sent and not self.cmd_bridge_active:
            self._send_setpoint(cur_x, cur_y, self.takeoff_z)
            return

        if self.goal_sent and self.cmd_bridge_active:
            self.transition_counter += 1
            if self.transition_counter == 1:
                self.get_logger().info(
                    "cmd_bridge detected — handing over control"
                )
            if self.transition_counter <= 20:
                self._send_setpoint(cur_x, cur_y, self.takeoff_z)
            elif self.transition_counter == 21:
                self.get_logger().info(
                    "Handover complete — cmd_bridge in control"
                )

        # ================================================================
        # Phase 6: Monitor progress (after handover)
        # ================================================================
        if self.cmd_bridge_active and self.transition_counter > 20:
            dist = (
                (cur_x - self.goal_x) ** 2
                + (cur_y - self.goal_y) ** 2
                + (cur_z - self.goal_z) ** 2
            ) ** 0.5
            if dist < 0.5:
                self.get_logger().info(
                    f"Goal reached! Distance: {dist:.2f}m"
                )
            elif self.transition_counter % 40 == 21:
                self.get_logger().info(
                    f"En route... dist to goal: {dist:.2f}m"
                )


def main():
    rclpy.init(args=sys.argv)
    goal_x = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    goal_y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    goal_z = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    takeoff_z = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0

    node = MissionControl(goal_x, goal_y, goal_z, takeoff_z)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
