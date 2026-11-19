#!/usr/bin/env python3
"""
Tag-scanning and precision landing mission controller.

Runs AFTER ego-planner completes all competition waypoints.
The drone is already at the landing zone (last waypoint = landing area).
This node takes over for: scan AprilTag -> PID-controlled precision landing.

State machine:
  WAIT_TRIGGER -> HOVER_SCAN -> TAG_DESCENT -> LANDED
                               -> TIMEOUT -> RETRY_CLIMB -> HOVER_SCAN
                                          -> RTL

Trigger: /mission/stage == "complete" (from competition mission controller)
         or auto_start parameter set to true.

Usage:
  ros2 run scouting_bridge tag_mission_controller.py
  ros2 run scouting_bridge tag_mission_controller.py --ros-args \
      -p target_tag_id:=7 \
      -p scan_timeout:=45.0 \
      -p land_height:=0.3
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry

# MAVROS messages - fall back if not installed (e.g. in sim without MAVROS)
try:
    from mavros_msgs.msg import State, PositionTarget
    from mavros_msgs.srv import SetMode, CommandBool
    _HAS_MAVROS = True
except ImportError:
    _HAS_MAVROS = False


class TagMissionController(Node):

    # --- States ----------------------------------------------------
    (
        WAIT_TRIGGER,
        HOVER_SCAN,
        RETRY_CLIMB,
        TAG_DESCENT,
        LANDED,
        TIMEOUT,
        RTL,
        ABORT,
    ) = range(8)

    _STATE_NAMES = {
        WAIT_TRIGGER:  "WAIT_TRIGGER",
        HOVER_SCAN:    "HOVER_SCAN",
        RETRY_CLIMB:   "RETRY_CLIMB",
        TAG_DESCENT:   "TAG_DESCENT",
        LANDED:        "LANDED",
        TIMEOUT:       "TIMEOUT",
        RTL:           "RTL",
        ABORT:         "ABORT",
    }

    # --- Descent sub-states ----------------------------------------
    (
        DESC_LOCK_XY,    # Horizontal alignment to tag
        DESC_LOWER,      # Descending one step
        DESC_FINAL,      # Final descent to landing
    ) = range(3)

    def __init__(self):
        super().__init__("tag_mission_controller")

        # --- Parameters --------------------------------------------
        self._declare_params()

        # --- State -------------------------------------------------
        self._state = self.WAIT_TRIGGER
        self._descent_sub_state = self.DESC_LOCK_XY
        self._state_enter_time = self.get_clock().now()

        self._mavros_odom: Odometry | None = None
        self._slam_odom: Odometry | None = None
        self._tag_pose: PoseStamped | None = None
        self._tag_id: int | None = None
        self._tag_lost = False

        self._armed = False
        self._connected = False
        self._mode = ""

        self._descent_stable_counter = 0
        self._retry_count = 0
        self._last_logged_elapsed = -1
        self._target_z = 0.0

        # Hover anchor: position captured when entering HOVER_SCAN
        self._hover_x = 0.0
        self._hover_y = 0.0
        self._hover_z = 0.0

        # --- PID state ---------------------------------------------
        self._pid_integral_x = 0.0
        self._pid_integral_y = 0.0
        self._pid_last_x_err = 0.0
        self._pid_last_y_err = 0.0
        self._pid_last_time = None

        # --- QoS ---------------------------------------------------
        _best_effort = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # --- Subscriptions -----------------------------------------
        # MAVROS odom: armed/connected state, altitude fallback
        self._mavros_odom_sub = self.create_subscription(
            Odometry, "/mavros/local_position/odom",
            self._mavros_odom_cb, _best_effort)

        # SLAM odom: camera_init frame, matches tag pose coordinate system
        self._slam_odom_sub = self.create_subscription(
            Odometry, "/Odometry",
            self._slam_odom_cb, _best_effort)

        self._tag_pose_sub = self.create_subscription(
            PoseStamped, "/tag_detection/pose",
            self._tag_pose_cb, 10)

        self._tag_id_sub = self.create_subscription(
            Int32, "/tag_detection/id",
            self._tag_id_cb, 10)

        self._stage_sub = self.create_subscription(
            String, "/mission/stage",
            self._stage_cb, 10)

        if _HAS_MAVROS:
            self._state_sub = self.create_subscription(
                State, "/mavros/state",
                self._state_cb, _best_effort)

        # --- Publishers --------------------------------------------
        self._stage_pub = self.create_publisher(
            String, "/mission/stage", 10)
        self._setpoint_pos_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        if _HAS_MAVROS:
            self._setpoint_raw_pub = self.create_publisher(
                PositionTarget, "/mavros/setpoint_raw/local", 10)

        # --- Service clients (MAVROS) ------------------------------
        self._set_mode_cli = None
        self._arm_cli = None
        if _HAS_MAVROS:
            self._set_mode_cli = self.create_client(
                SetMode, "/mavros/set_mode")
            self._arm_cli = self.create_client(
                CommandBool, "/mavros/cmd/arming")

        # --- Control loop (10 Hz) ----------------------------------
        self._timer = self.create_timer(0.1, self._control_loop)

        # --- Setpoint stream (20 Hz, must run continuously) --------
        self._setpoint_timer = self.create_timer(0.05, self._setpoint_stream)

        self.get_logger().info(
            f"TagMissionController initialized. "
            f"Target tag ID: {self._target_tag_id}. "
            f"Trigger: mission complete or auto-start. "
            f"AUTO_LAND={'ON' if self._auto_land else 'OFF'}."
        )

    # =================================================================
    # Parameters
    # =================================================================
    def _declare_params(self):
        # Trigger
        self.declare_parameter("auto_start", False)
        self.declare_parameter("target_tag_id", 0)  # 0 = any tag

        # Scan
        self.declare_parameter("scan_timeout", 30.0)

        # Retry
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("retry_climb", 1.0)   # extra height per retry

        # Descent
        self.declare_parameter("descend_step", 0.5)
        self.declare_parameter("land_height", 0.3)
        self.declare_parameter("descent_velocity", 0.3)  # m/s

        # PID for horizontal control during descent
        self.declare_parameter("pid_kp_xy", 0.5)
        self.declare_parameter("pid_ki_xy", 0.02)
        self.declare_parameter("pid_kd_xy", 0.15)
        self.declare_parameter("max_horizontal_vel", 1.0)
        self.declare_parameter("tag_lock_dist", 0.15)  # meters

        # Landing
        self.declare_parameter("auto_land", True)

        # Read into instance vars
        self._auto_start     = self.get_parameter("auto_start").value
        self._target_tag_id  = self.get_parameter("target_tag_id").value
        self._scan_timeout   = self.get_parameter("scan_timeout").value
        self._max_retries    = self.get_parameter("max_retries").value
        self._retry_climb    = self.get_parameter("retry_climb").value
        self._descend_step   = self.get_parameter("descend_step").value
        self._land_height    = self.get_parameter("land_height").value
        self._descent_vel    = self.get_parameter("descent_velocity").value
        self._kp             = self.get_parameter("pid_kp_xy").value
        self._ki             = self.get_parameter("pid_ki_xy").value
        self._kd             = self.get_parameter("pid_kd_xy").value
        self._max_hvel       = self.get_parameter("max_horizontal_vel").value
        self._tag_lock_dist  = self.get_parameter("tag_lock_dist").value
        self._auto_land      = self.get_parameter("auto_land").value

        # Setpoint state
        self._sp_x = 0.0
        self._sp_y = 0.0
        self._sp_z = 0.0
        self._sp_vx = 0.0
        self._sp_vy = 0.0
        self._sp_yaw = 0.0
        self._use_raw_setpoint = False

    # =================================================================
    # Callbacks
    # =================================================================
    def _mavros_odom_cb(self, msg: Odometry):
        """MAVROS local odom: used for yaw extraction and fallback position."""
        self._mavros_odom = msg
        q = msg.pose.pose.orientation
        self._sp_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _slam_odom_cb(self, msg: Odometry):
        """SLAM odom (camera_init frame): used for PID error computation
        because tag pose is also in camera_init frame."""
        self._slam_odom = msg

    def _tag_pose_cb(self, msg: PoseStamped):
        self._tag_pose = msg

    def _tag_id_cb(self, msg: Int32):
        self._tag_id = msg.data
        self._tag_lost = (msg.data < 0)

    def _stage_cb(self, msg: String):
        if msg.data == "complete":
            self._try_trigger("mission_complete")

    def _state_cb(self, msg):
        self._connected = msg.connected
        self._armed = msg.armed
        self._mode = msg.mode

    # =================================================================
    # Setpoint stream (20 Hz)
    # =================================================================
    def _setpoint_stream(self):
        """Continuously publish setpoints so drone stays in OFFBOARD.

        During WAIT_TRIGGER we do NOT publish — cmd_bridge is active.
        After trigger, we take over the setpoint stream immediately.
        """
        now = self.get_clock().now().to_msg()

        if self._state == self.WAIT_TRIGGER:
            return  # cmd_bridge + competition controller are in control

        if self._use_raw_setpoint and _HAS_MAVROS:
            self._publish_raw_setpoint(now)
        else:
            self._publish_position_setpoint(now)

    def _publish_position_setpoint(self, stamp):
        pt = PoseStamped()
        pt.header.stamp = stamp
        pt.pose.position.x = float(self._sp_x)
        pt.pose.position.y = float(self._sp_y)
        pt.pose.position.z = float(self._sp_z)
        pt.pose.orientation.w = math.cos(self._sp_yaw * 0.5)
        pt.pose.orientation.z = math.sin(self._sp_yaw * 0.5)
        self._setpoint_pos_pub.publish(pt)

    def _publish_raw_setpoint(self, stamp):
        if not _HAS_MAVROS:
            return
        pt = PositionTarget()
        pt.header.stamp = stamp
        # FRAME_LOCAL_NED — despite the name, MAVROS expects ENU input
        # and internally applies enu→ned transform. Do NOT pre-convert.
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Velocity-controlled XY + position-controlled Z
        pt.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        # ── ENU (NO manual conversion — MAVROS handles it) ──
        pt.position.z = float(self._sp_z)
        pt.velocity.x = float(self._sp_vx)
        pt.velocity.y = float(self._sp_vy)
        pt.yaw = float(self._sp_yaw)
        self._setpoint_raw_pub.publish(pt)

    # =================================================================
    # Control loop (10 Hz) - state machine
    # =================================================================
    def _control_loop(self):
        if self._state == self.WAIT_TRIGGER:
            self._run_wait_trigger()
        elif self._state == self.HOVER_SCAN:
            self._run_hover_scan()
        elif self._state == self.RETRY_CLIMB:
            self._run_retry_climb()
        elif self._state == self.TAG_DESCENT:
            self._run_tag_descent()
        elif self._state == self.LANDED:
            self._run_landed()
        elif self._state == self.TIMEOUT:
            self._run_timeout()
        elif self._state == self.RTL:
            self._run_rtl()

    # --- WAIT_TRIGGER ----------------------------------------------
    def _try_trigger(self, reason: str):
        if self._state != self.WAIT_TRIGGER:
            return
        if self._mavros_odom is None:
            self.get_logger().warn(
                f"Cannot start ({reason}): no odom yet")
            return
        # Capture current position as hover anchor
        p = self._mavros_odom.pose.pose.position
        self._hover_x = p.x
        self._hover_y = p.y
        self._hover_z = p.z
        self._sp_x = self._hover_x
        self._sp_y = self._hover_y
        self._sp_z = self._hover_z
        self._transition(self.HOVER_SCAN)

        # SYNC publish: fill the gap immediately — don't wait for the
        # 20 Hz timer. competition controller stops setpoints on
        # "complete" and this keeps PX4 in OFFBOARD without a hiccup.
        self._publish_position_setpoint(self.get_clock().now().to_msg())

        self.get_logger().info(
            f"Mission started ({reason}). "
            f"Hovering at ({self._hover_x:.2f}, {self._hover_y:.2f}, {self._hover_z:.2f})"
        )

    def _run_wait_trigger(self):
        if self._auto_start and self._mavros_odom is not None:
            self._try_trigger("auto_start")

    # --- HOVER_SCAN ------------------------------------------------
    def _run_hover_scan(self):
        """Hover at the position where ego-planner left us, scan for tag."""
        self._use_raw_setpoint = False
        # Hold position
        self._sp_x = self._hover_x
        self._sp_y = self._hover_y
        self._sp_z = self._hover_z + self._retry_count * self._retry_climb

        elapsed = (self.get_clock().now() -
                   self._state_enter_time).nanoseconds / 1e9

        # Tag detection check
        if self._tag_detected():
            self.get_logger().info(
                f"Tag {self._tag_id} locked at "
                f"({self._tag_pose.pose.position.x:.2f}, "
                f"{self._tag_pose.pose.position.y:.2f}, "
                f"{self._tag_pose.pose.position.z:.2f})"
            )
            self._transition(self.TAG_DESCENT)
            return

        # Timeout
        if elapsed > self._scan_timeout:
            self.get_logger().warn(
                f"Scan timeout after {elapsed:.1f}s")
            self._transition(self.TIMEOUT)

        # Periodic status (throttled to once every 5s)
        elapsed_int = int(elapsed)
        if elapsed_int % 5 == 0 and elapsed_int != self._last_logged_elapsed:
            self._last_logged_elapsed = elapsed_int
            odom_z = (self._mavros_odom.pose.pose.position.z
                      if self._mavros_odom is not None else float('nan'))
            self.get_logger().info(
                f"Scanning... {elapsed:.0f}s. "
                f"Height: {odom_z:.2f}m. "
                f"Tag seen: {self._tag_id}"
            )

    def _tag_detected(self) -> bool:
        """Check if the target tag is currently visible."""
        if self._tag_id is None or self._tag_id < 0:
            return False
        if self._target_tag_id > 0 and self._tag_id != self._target_tag_id:
            return False
        return self._tag_pose is not None

    # --- RETRY_CLIMB -----------------------------------------------
    def _run_retry_climb(self):
        """Climb to expand FOV before retrying scan."""
        self._use_raw_setpoint = False
        target_z = self._hover_z + self._retry_count * self._retry_climb
        # Ramp toward target (smooth climb)
        if self._mavros_odom is not None:
            pz = self._mavros_odom.pose.pose.position.z
            self._sp_z = pz + 0.2  # climb at ~2 m/s
            self._sp_z = min(self._sp_z, target_z)
        self._sp_x = self._hover_x
        self._sp_y = self._hover_y

        if self._mavros_odom is not None and abs(
                self._mavros_odom.pose.pose.position.z - target_z) < 0.2:
            self.get_logger().info(
                f"Climbed to {target_z:.2f}m, resuming scan")
            self._transition(self.HOVER_SCAN)

    # --- TAG_DESCENT -----------------------------------------------
    def _run_tag_descent(self):
        """PID-controlled descent onto detected tag."""
        self._use_raw_setpoint = True
        # Use SLAM odom (camera_init frame) for PID: same frame as tag pose.
        # Fall back to MAVROS odom if SLAM is unavailable.
        odom = self._slam_odom if self._slam_odom is not None else self._mavros_odom
        if odom is None:
            return

        pz = odom.pose.pose.position.z

        # Horizontal PID on tag position
        if self._tag_pose is not None and not self._tag_lost:
            err_x = self._tag_pose.pose.position.x - odom.pose.pose.position.x
            err_y = self._tag_pose.pose.position.y - odom.pose.pose.position.y
        else:
            err_x = self._sp_x - odom.pose.pose.position.x
            err_y = self._sp_y - odom.pose.pose.position.y

        now = self.get_clock().now().nanoseconds / 1e9
        dt = 0.1
        if self._pid_last_time is not None:
            dt = max(now - self._pid_last_time, 0.01)
        self._pid_last_time = now

        # PID X
        self._pid_integral_x += err_x * dt
        deriv_x = (err_x - self._pid_last_x_err) / dt
        vx = self._kp * err_x + self._ki * self._pid_integral_x + self._kd * deriv_x
        vx = max(-self._max_hvel, min(self._max_hvel, vx))
        self._pid_last_x_err = err_x

        # PID Y
        self._pid_integral_y += err_y * dt
        deriv_y = (err_y - self._pid_last_y_err) / dt
        vy = self._kp * err_y + self._ki * self._pid_integral_y + self._kd * deriv_y
        vy = max(-self._max_hvel, min(self._max_hvel, vy))
        self._pid_last_y_err = err_y

        self._sp_vx = vx
        self._sp_vy = vy

        # Cache tag world position for fallback
        if self._tag_pose is not None:
            self._sp_x = self._tag_pose.pose.position.x
            self._sp_y = self._tag_pose.pose.position.y

        horizontal_err = math.hypot(err_x, err_y)

        # Descent sub-state machine
        if self._descent_sub_state == self.DESC_LOCK_XY:
            if horizontal_err < self._tag_lock_dist:
                self._descent_stable_counter += 1
            else:
                self._descent_stable_counter = 0

            if self._descent_stable_counter >= 10:  # 1s stable
                self._target_z = pz - self._descend_step
                self._descent_sub_state = self.DESC_LOWER
                self._descent_stable_counter = 0
                self.get_logger().info(
                    f"Descending to {self._target_z:.2f}m "
                    f"(err_xy={horizontal_err:.2f}m)")

        elif self._descent_sub_state == self.DESC_LOWER:
            self._sp_z = max(self._land_height,
                           pz - self._descent_vel * 0.1)

            if pz <= self._target_z + 0.1:
                self._descent_stable_counter += 1
            else:
                self._descent_stable_counter = 0

            if self._descent_stable_counter >= 5:
                if pz <= self._land_height + 1.0:
                    self._descent_sub_state = self.DESC_FINAL
                    self.get_logger().info("Final descent phase")
                else:
                    self._descent_sub_state = self.DESC_LOCK_XY
                    self._descent_stable_counter = 0

        elif self._descent_sub_state == self.DESC_FINAL:
            self._sp_z = max(0.0, pz - self._descent_vel * 0.1)

            if pz <= self._land_height:
                self.get_logger().info(
                    f"Land height reached ({pz:.2f}m)")
                self._transition(self.LANDED)

    # --- LANDED ----------------------------------------------------
    def _run_landed(self):
        """Switch to AUTO.LAND mode, then disarm after descent."""
        if not self._auto_land:
            self._timer.cancel()
            self._setpoint_timer.cancel()
            return

        if _HAS_MAVROS and self._set_mode_cli is not None:
            self.get_logger().info("Switching to AUTO.LAND...")
            self._set_mode_cli.call_async(
                SetMode.Request(custom_mode="AUTO.LAND"))
        else:
            self.get_logger().warn("MAVROS unavailable — falling back to disarm")
            self._disarm()

        # Stop all setpoint streaming; PX4 handles the landing
        self._timer.cancel()
        self._setpoint_timer.cancel()

    # --- TIMEOUT ---------------------------------------------------
    def _run_timeout(self):
        self._retry_count += 1
        if self._retry_count <= self._max_retries:
            self.get_logger().warn(
                f"Retry {self._retry_count}/{self._max_retries}: "
                f"climbing {self._retry_climb}m higher")
            self._transition(self.RETRY_CLIMB)
        else:
            self.get_logger().error(
                f"Max retries ({self._max_retries}) exceeded. "
                f"RTL.")
            self._transition(self.RTL)

    # --- RTL --------------------------------------------------------
    def _run_rtl(self):
        """Switch to AUTO.RTL mode and let PX4 handle the return."""
        self._use_raw_setpoint = False
        if not _HAS_MAVROS or self._set_mode_cli is None:
            self.get_logger().error("Cannot RTL: MAVROS unavailable")
            return
        self.get_logger().info("Switching to AUTO.RTL...")
        self._set_mode_cli.call_async(
            SetMode.Request(custom_mode="AUTO.RTL"))
        self._publish_stage("rtl")
        # Stop further processing — PX4 takes over
        self._timer.cancel()
        self._setpoint_timer.cancel()

    # =================================================================
    # Helpers
    # =================================================================
    def _transition(self, new_state: int):
        old_name = self._STATE_NAMES.get(self._state, "?")
        new_name = self._STATE_NAMES.get(new_state, "?")
        self.get_logger().info(f"State: {old_name} -> {new_name}")
        self._state = new_state
        self._state_enter_time = self.get_clock().now()
        self._descent_stable_counter = 0

        # Reset PID on state transitions
        self._pid_integral_x = 0.0
        self._pid_integral_y = 0.0
        self._pid_last_x_err = 0.0
        self._pid_last_y_err = 0.0
        self._pid_last_time = None

        self._publish_stage(new_name)

    def _publish_stage(self, stage: str):
        self._stage_pub.publish(String(data=stage))

    def _disarm(self):
        if not _HAS_MAVROS or self._arm_cli is None:
            self.get_logger().warn("Cannot disarm: MAVROS unavailable")
            return
        if not self._arm_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Arming service unavailable")
            return
        req = CommandBool.Request()
        req.value = False
        future = self._arm_cli.call_async(req)

        def _disarm_done(fut):
            try:
                resp = fut.result()
                self.get_logger().info(
                    f"Disarm {'successful' if resp.success else 'FAILED'}")
            except Exception as e:
                self.get_logger().error(f"Disarm call failed: {e}")

        future.add_done_callback(_disarm_done)


def main():
    rclpy.init()
    node = TagMissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
