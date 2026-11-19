#!/usr/bin/env python3
"""
Competition mission controller for ego-planner PRESET_TARGET mode.

Architecture:
  - All waypoints are pre-loaded into ego_planner_node as FSM parameters
    (done by ego_planner_competition.launch.py from competition_waypoints.yaml)
  - This node handles: init (OFFBOARD → arm → takeoff) → trigger → handover → monitor
  - FSM auto-advances through waypoints after receiving /traj_start_trigger
  - Detects cmd_bridge takeover via /position_cmd (PositionCommand)
  - Monitors /mavros/local_position/odom to track stage transitions and timing

Phases:
  1. OFFBOARD mode switch
  2. Arm
  3. Takeoff to target height
  4. Send /traj_start_trigger to FSM
  5. Handover: maintain hover setpoints until cmd_bridge confirms active + 2s overlap
  6. Monitor: track waypoint arrival, stage transitions, C-stage height limit

Strategy:
  - B stage: Strategy 1 (pass-through only)
  - D stage: Fixed ring (100 pts)

Usage:
  ros2 run ego_planner mission_controller_competition.py
  ros2 run ego_planner mission_controller_competition.py --ros-args -p waypoints_yaml:=/path/to/yaml
"""

import os
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand

# Optional MAVROS imports — graceful fallback if unavailable
try:
    from mavros_msgs.msg import State
    from mavros_msgs.srv import SetMode, CommandBool
    _HAS_MAVROS = True
except ImportError:
    _HAS_MAVROS = False


class MissionController(Node):
    def __init__(self, waypoints_yaml: str):
        super().__init__("mission_controller_competition")

        # ── Load waypoints config ──────────────────────────────────
        with open(waypoints_yaml, 'r') as f:
            self._config = yaml.safe_load(f)

        self._waypoints = self._flatten_waypoints()
        self._total_wp = len(self._waypoints)
        self.get_logger().info(
            f"Loaded {self._total_wp} waypoints from {waypoints_yaml}"
        )
        for i, wp in enumerate(self._waypoints):
            self.get_logger().info(
                f"  wp{i} [{wp['stage']}]: ({wp['x']:.1f}, {wp['y']:.1f}, {wp['z']:.1f})"
            )

        # ── State ──────────────────────────────────────────────────
        self._odom = None          # latest Odometry
        self._connected = False    # MAVROS connection
        self._armed = False
        self._mode = ""
        self._trigger_sent = False
        self._init_complete = False
        self._cmd_bridge_active = False
        self._handover_counter = 0
        self._mission_complete = False
        self._reached_wps = [False] * self._total_wp
        self._current_stage = "init"
        self._stage_enter_time = self.get_clock().now()
        self._stage_times = {}     # stage → elapsed seconds
        self._start_time = None    # trigger time for total duration


        # ── QoS ────────────────────────────────────────────────────
        _qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ── Subscriptions ──────────────────────────────────────────
        self._odom_sub = self.create_subscription(
            Odometry, "/mavros/local_position/odom", self._odom_cb, _qos)
        self._cmd_sub = self.create_subscription(
            PositionCommand, "/position_cmd", self._cmd_cb, 10)

        if _HAS_MAVROS:
            self._state_sub = self.create_subscription(
                State, "/mavros/state", self._state_cb, _qos)

        # ── Publishers ─────────────────────────────────────────────
        self._trigger_pub = self.create_publisher(
            PoseStamped, "/traj_start_trigger", 10)
        self._stage_pub = self.create_publisher(
            String, "/mission/stage", 10)
        # ── Service clients (MAVROS) ────────────────────────────────
        if _HAS_MAVROS:
            self._set_mode_cli = self.create_client(SetMode, "/mavros/set_mode")
            self._arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
        else:
            self._set_mode_cli = None
            self._arm_cli = None

        # ── Setpoint publisher (for takeoff phase) ──────────────────
        if _HAS_MAVROS:
            self._setpoint_pub = self.create_publisher(
                PoseStamped, "/mavros/setpoint_position/local", 10)
        else:
            self._setpoint_pub = None

        # ── Control loop ───────────────────────────────────────────
        self._timer = self.create_timer(0.1, self._control_loop)

        # Retry state
        self._mode_attempts = 0
        self._arm_attempts = 0
        self._takeoff_stable_count = 0
        self._takeoff_xy_recorded = False
        self._takeoff_xy_x = 0.0
        self._takeoff_xy_y = 0.0
        self._settle_count = 0
        self._cmd_bridge_ready_count = 0
        self._cmd_bridge_proc = None
        self._max_retries = 50
        self._no_odom_count = 0
        self._no_fcu_count = 0

        self.get_logger().info(
            f"MissionController ready "
            f"(mavros={'available' if _HAS_MAVROS else 'unavailable'})"
        )

    # ── Helpers ────────────────────────────────────────────────────
    def _flatten_waypoints(self) -> list:
        """Flatten all stage waypoints into a single ordered list."""
        wps = []
        for stage_name in ['A', 'B', 'C', 'D']:
            stage = self._config['stages'][stage_name]
            for wp in stage['waypoints']:
                wps.append({
                    'x': float(wp[0]),
                    'y': float(wp[1]),
                    'z': float(wp[2]),
                    'stage': stage_name,
                })
        return wps

    def _next_unreached(self) -> int:
        """Return index of first unreached waypoint, or -1 if all done."""
        for i, reached in enumerate(self._reached_wps):
            if not reached:
                return i
        return -1

    def _get_stage_name(self, wp_idx: int) -> str:
        if wp_idx < 0 or wp_idx >= self._total_wp:
            return "complete"
        return self._waypoints[wp_idx]['stage']

    def _publish_stage(self, stage: str):
        if stage != self._current_stage:
            # Record time elapsed in previous stage
            elapsed = (self.get_clock().now() - self._stage_enter_time).nanoseconds / 1e9
            self._stage_times[self._current_stage] = elapsed
            self.get_logger().info(
                f"Stage transition: {self._current_stage} → {stage} "
                f"({elapsed:.1f}s)"
            )
            self._current_stage = stage
            self._stage_enter_time = self.get_clock().now()

        msg = String()
        msg.data = stage
        self._stage_pub.publish(msg)

    # ── Callbacks ──────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    def _cmd_cb(self, msg: PositionCommand):
        self._cmd_bridge_active = True

    def _state_cb(self, msg):
        self._connected = msg.connected
        self._armed = msg.armed
        self._mode = msg.mode

    # ── MAVROS helpers ─────────────────────────────────────────────
    def _send_setpoint(self, x: float, y: float, z: float):
        if self._setpoint_pub is None:
            return
        pt = PoseStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.pose.position.x = float(x)
        pt.pose.position.y = float(y)
        pt.pose.position.z = float(z)
        pt.pose.orientation.w = 1.0
        self._setpoint_pub.publish(pt)

    def _try_offboard(self) -> bool:
        if self._mode == "OFFBOARD":
            self._mode_attempts = 0
            return True
        if self._mode_attempts < self._max_retries:
            if self._mode_attempts == 0:
                self.get_logger().info(f"Switching {self._mode} → OFFBOARD...")
            self._set_mode_cli.call_async(
                SetMode.Request(custom_mode="OFFBOARD"))
            self._mode_attempts += 1
        return False

    def _try_arm(self) -> bool:
        if self._armed:
            self._arm_attempts = 0
            return True
        if self._arm_attempts < self._max_retries:
            if self._arm_attempts == 0:
                self.get_logger().info("Arming...")
            self._arm_cli.call_async(CommandBool.Request(value=True))
            self._arm_attempts += 1
        return False

    # ── Main loop ──────────────────────────────────────────────────
    def _control_loop(self):
        """Main FSM at 10 Hz."""
        odom = self._odom

        # ── Fallback: if no MAVROS, skip init and go straight to trigger ──
        if not _HAS_MAVROS:
            if not self._trigger_sent and odom is not None:
                self._send_trigger(odom)
            if self._trigger_sent:
                self._monitor_progress(odom)
            return

        # ── Stream setpoints even before connection ──
        if odom is None:
            self._send_setpoint(0.0, 0.0, 0.0)
            self._no_odom_count += 1
            if self._no_odom_count == 1 or self._no_odom_count % 50 == 0:
                self.get_logger().warn(
                    f"Waiting for /mavros/local_position/odom... "
                    f"({self._no_odom_count / 10:.0f}s)"
                )
            return

        cur_x = odom.pose.pose.position.x
        cur_y = odom.pose.pose.position.y
        cur_z = odom.pose.pose.position.z

        # ── Phase 1: OFFBOARD ──────────────────────────────────────
        if not self._connected:
            self._send_setpoint(0.0, 0.0, 0.0)
            self._no_fcu_count += 1
            if self._no_fcu_count == 1 or self._no_fcu_count % 50 == 0:
                self.get_logger().warn(
                    f"FCU not connected — waiting for /mavros/state... "
                    f"({self._no_fcu_count / 10:.0f}s)"
                )
            return

        if not self._armed and self._mode != "OFFBOARD":
            self._send_setpoint(0.0, 0.0, 0.0)
            self._try_offboard()
            self._publish_stage("init")
            return

        # ── Phase 2: Arm ───────────────────────────────────────────
        if self._mode == "OFFBOARD" and not self._armed:
            self._send_setpoint(0.0, 0.0, 0.0)
            self._try_arm()
            self._publish_stage("init")
            return

        # ── Phase 3: Takeoff ───────────────────────────────────────
        takeoff_z = self._config.get('defaults', {}).get('takeoff_height', 1.5)
        if self._armed and self._mode == "OFFBOARD" and not self._init_complete:

            # Record XY position on first takeoff entry for drift tracking
            if not self._takeoff_xy_recorded:
                self._takeoff_xy_x = cur_x
                self._takeoff_xy_y = cur_y
                self._takeoff_xy_recorded = True
                self.get_logger().info(
                    f"Takeoff position locked: ({cur_x:.2f}, {cur_y:.2f})"
                )

            # Use recorded XY as setpoint — take off straight up from wherever we are
            self._send_setpoint(self._takeoff_xy_x, self._takeoff_xy_y, takeoff_z)

            # Require BOTH Z height AND XY drift within tolerance
            xy_drift = ((cur_x - self._takeoff_xy_x) ** 2
                        + (cur_y - self._takeoff_xy_y) ** 2) ** 0.5
            z_ok = abs(cur_z - takeoff_z) < 0.3
            xy_ok = xy_drift < 0.3

            if z_ok and xy_ok:
                self._takeoff_stable_count += 1
                if self._takeoff_stable_count > 30:  # ~3s stable at 10Hz
                    self.get_logger().info(
                        f"Takeoff hover confirmed at z={cur_z:.2f}m, "
                        f"xy_drift={xy_drift:.2f}m"
                    )
                    self._init_complete = True
            else:
                self._takeoff_stable_count = 0
                self._settle_count = 0
            self._publish_stage("init")
            return

        # ── Phase 3.5: Post-hover settling before trigger ──────────
        if self._init_complete and not self._trigger_sent \
                and self._settle_count < 10:
            takeoff_z = self._config.get('defaults', {}).get('takeoff_height', 1.5)
            self._send_setpoint(cur_x, cur_y, takeoff_z)
            self._settle_count += 1
            if self._settle_count == 1:
                self.get_logger().info(
                    "Hover confirmed — settling 1s before waypoint trigger..."
                )
            return

        # ── Phase 3.6: Start cmd_bridge after takeoff ───────────────
        # This phase runs AFTER settle completes and BEFORE trigger.
        # We launch cmd_bridge here so it never publishes setpoints
        # during takeoff — avoiding ground-level yaw/movement.
        if self._init_complete and not self._trigger_sent \
                and self._settle_count >= 10 \
                and self._cmd_bridge_ready_count < 30:
            if self._cmd_bridge_ready_count == 0:
                if not self._start_cmd_bridge():
                    self.get_logger().error(
                        "cmd_bridge failed to start — aborting mission"
                    )
                    self._mission_complete = True
                    return
            takeoff_z = self._config.get('defaults', {}).get('takeoff_height', 1.5)
            self._send_setpoint(cur_x, cur_y, takeoff_z)
            self._cmd_bridge_ready_count += 1
            if self._cmd_bridge_ready_count == 1:
                self.get_logger().info(
                    "cmd_bridge initializing — holding hover for 3s..."
                )
            return

        # ── Phase 4: Send trigger to FSM ───────────────────────────
        if self._init_complete and not self._trigger_sent:
            self._send_trigger(odom)
            self._cmd_bridge_active = False
            self._handover_counter = 0

        # ── Phase 5: Transition to cmd_bridge ──────────────────────
        if self._trigger_sent and not self._cmd_bridge_active:
            self._send_setpoint(cur_x, cur_y, takeoff_z)
            return

        if self._trigger_sent and self._cmd_bridge_active:
            self._handover_counter += 1
            if self._handover_counter <= 20:
                self._send_setpoint(cur_x, cur_y, takeoff_z)
                return
            elif self._handover_counter == 21:
                self.get_logger().info("Handover complete — cmd_bridge in control")

        # ── Phase 6: Monitor progress ───────────────────────────────
        if self._mission_complete:
            return

        if self._trigger_sent and self._cmd_bridge_active:
            self._monitor_progress(odom)

    def _start_cmd_bridge(self) -> bool:
        """Launch cmd_bridge via subprocess after takeoff is confirmed.

        cmd_bridge bridges /position_cmd -> /mavros/setpoint_position/local.
        By starting it only after takeoff, we guarantee no setpoint conflict
        during the OFFBOARD → arm → takeoff sequence, preventing ground-level
        yaw/movement ("磨地转向").
        """
        import subprocess
        self.get_logger().info("Starting cmd_bridge after takeoff...")
        try:
            self._cmd_bridge_proc = subprocess.Popen(
                ['ros2', 'run', 'ego_planner', 'cmd_bridge'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            # Brief wait for node to register subscriber/publisher
            time.sleep(2.0)
            if self._cmd_bridge_proc.poll() is not None:
                self.get_logger().error(
                    f"cmd_bridge exited immediately "
                    f"(code={self._cmd_bridge_proc.returncode})"
                )
                return False
            self.get_logger().info("cmd_bridge started successfully")
            return True
        except FileNotFoundError:
            self.get_logger().error(
                "'ros2' CLI not found in PATH — "
                "is the ROS 2 environment sourced?"
            )
            return False
        except Exception as e:
            self.get_logger().error(f"Failed to start cmd_bridge: {e}")
            return False

    def _send_trigger(self, odom):
        """Send /traj_start_trigger to FSM. Content is ignored, just needs to arrive."""
        trigger = PoseStamped()
        trigger.header.stamp = self.get_clock().now().to_msg()
        trigger.header.frame_id = "camera_init"
        trigger.pose.position.x = odom.pose.pose.position.x
        trigger.pose.position.y = odom.pose.pose.position.y
        trigger.pose.position.z = odom.pose.pose.position.z
        trigger.pose.orientation.w = 1.0
        self._trigger_pub.publish(trigger)
        self._trigger_sent = True
        self._start_time = self.get_clock().now()
        self.get_logger().info("Trigger sent — FSM waypoint execution started")

    def _monitor_progress(self, odom):
        """Track waypoint arrival and stage transitions."""
        if odom is None:
            self.get_logger().warn(
                "No odometry received — waiting for /Odometry...",
                throttle_duration_sec=5.0)
            return

        cur_x = odom.pose.pose.position.x
        cur_y = odom.pose.pose.position.y
        cur_z = odom.pose.pose.position.z

        # Check which waypoints have been reached
        complete_dist = self._config.get('defaults', {}).get('complete_dist', 0.5)
        for i, reached in enumerate(self._reached_wps):
            if reached:
                continue
            wp = self._waypoints[i]
            dist = ((cur_x - wp['x'])**2 + (cur_y - wp['y'])**2 + (cur_z - wp['z'])**2)**0.5
            if dist < complete_dist:
                self._reached_wps[i] = True
                self.get_logger().info(
                    f"Waypoint {i} [{wp['stage']}] reached (dist={dist:.2f}m)"
                )

        # Determine current stage
        next_wp = self._next_unreached()
        stage = self._get_stage_name(next_wp)
        self._publish_stage(stage)

        # ── Completion ──
        if stage == "complete":
            self._report_completion()

    def _report_completion(self):
        """Print final timing report."""
        if self._mission_complete:
            return
        self._mission_complete = True

        # ── Kill cmd_bridge to prevent setpoint conflicts ────────────
        # tag_mission_controller will take over /mavros/setpoint_position/local.
        # cmd_bridge must stop publishing before handover completes.
        import subprocess
        result = subprocess.run(
            ['pkill', '-f', 'ego_planner.*cmd_bridge'],
            capture_output=True, text=True)
        if result.returncode == 0:
            self.get_logger().info("cmd_bridge killed — handover to tag_mission")
        else:
            self.get_logger().warn(
                f"cmd_bridge kill returned {result.returncode} "
                f"(may already be stopped)")

        total = (self.get_clock().now() - self._start_time).nanoseconds / 1e9 if self._start_time else 0.0

        self.get_logger().info("=" * 50)
        self.get_logger().info(" COMPETITION COMPLETE")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"  Total time: {total:.1f}s")
        for stage in ['A', 'B', 'C', 'D']:
            t = self._stage_times.get(stage, 0.0)
            self.get_logger().info(f"  Stage {stage}: {t:.1f}s")
        self.get_logger().info("=" * 50)

        # Publish completion
        msg = String()
        msg.data = "complete"
        self._stage_pub.publish(msg)



def main():
    rclpy.init(args=sys.argv)

    # Resolve waypoints yaml path
    node_for_params = rclpy.create_node('_mission_controller_params')
    node_for_params.declare_parameter('waypoints_yaml', '')

    # Also check __file__-relative path (workspace dev)
    _script_dir = os.path.dirname(os.path.realpath(__file__))
    _ws_yaml = os.path.abspath(os.path.join(
        _script_dir, '..', '..', '..', '..', 'config', 'competition_waypoints.yaml'))

    yaml_path = node_for_params.get_parameter('waypoints_yaml').value
    if not yaml_path or not os.path.exists(yaml_path):
        if os.path.exists(_ws_yaml):
            yaml_path = _ws_yaml
        else:
            node_for_params.get_logger().fatal(
                f"waypoints_yaml not found. Tried:\n"
                f"  param: {yaml_path}\n"
                f"  workspace: {_ws_yaml}\n"
                f"Usage: ros2 run ego_planner mission_controller_competition.py "
                f"--ros-args -p waypoints_yaml:=/path/to/yaml"
            )
            node_for_params.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    node_for_params.destroy_node()

    node = MissionController(yaml_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
