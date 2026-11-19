#!/usr/bin/env python3
"""
Bridge ego-planner PositionCommand to MAVROS setpoint_raw/local.

Ego-planner publishes /position_cmd (quadrotor_msgs/PositionCommand) at 100Hz in ENU frame.
MAVROS /mavros/setpoint_raw/local expects mavros_msgs/PositionTarget.

Coordinate frame: FRAME_LOCAL_ENU (ROS convention: x=East, y=North, z=Up)
Type mask: use position + velocity + yaw (ignore acceleration, yaw rate)

This runs at the same rate as PositionCommand (100Hz) which exceeds
PX4's minimum 2Hz requirement for offboard mode.
"""
import sys
import rclpy
from rclpy.node import Node
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget


# Ignore acceleration (af) and yaw_rate, use position + velocity + yaw
TYPE_MASK = (
    PositionTarget.IGNORE_AFX
    | PositionTarget.IGNORE_AFY
    | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW_RATE
)


class CmdBridge(Node):
    def __init__(self, cmd_topic: str = "/position_cmd"):
        super().__init__("cmd_bridge")

        # Subscribe to ego-planner position commands
        self._cmd_sub = self.create_subscription(
            PositionCommand, cmd_topic, self._cmd_callback, 10
        )

        # Publish to MAVROS offboard setpoint
        self._setpoint_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )

        self.get_logger().info(
            f"Bridging {cmd_topic} -> /mavros/setpoint_raw/local [FRAME_LOCAL_ENU]"
        )

    def _cmd_callback(self, cmd: PositionCommand):
        target = PositionTarget()
        target.header.stamp = cmd.header.stamp
        target.header.frame_id = "map"  # MAVROS uses 'map' for local frame

        # FRAME_LOCAL_ENU: x=East, y=North, z=Up (matches ROS convention)
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_ENU

        # Use position + velocity + yaw (let PX4 handle acceleration control)
        target.type_mask = TYPE_MASK

        # Position
        target.position.x = cmd.position.x
        target.position.y = cmd.position.y
        target.position.z = cmd.position.z

        # Velocity feed-forward
        target.velocity.x = cmd.velocity.x
        target.velocity.y = cmd.velocity.y
        target.velocity.z = cmd.velocity.z

        # Yaw (heading)
        target.yaw = float(cmd.yaw)

        self._setpoint_pub.publish(target)


def main():
    rclpy.init(args=sys.argv)
    cmd_topic = sys.argv[1] if len(sys.argv) > 1 else "/position_cmd"
    node = CmdBridge(cmd_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
