/**
 * cmd_bridge.cpp — Bridge ego-planner PositionCommand to MAVROS setpoint_raw/local.
 *
 * Ego-planner publishes /position_cmd (quadrotor_msgs/PositionCommand) at 100Hz
 * in ROS ENU convention (x=East, y=North, z=Up).
 *
 * Key insight from mavros source (setpoint_raw.cpp):
 *   Despite the constant name FRAME_LOCAL_NED, MAVROS expects ENU input and
 *   internally applies transform_frame_enu_ned() before sending to the FCU.
 *   DO NOT manually convert ENU→NED — that would be a double conversion.
 *
 *   References:
 *     - mavlink/mavros Issue #1500 (coordinate frame doc confusion)
 *     - mavlink/mavros Issue #1009 (yaw transform bug, fixed in later versions)
 *     - mavros setpoint_raw.cpp: send_position_target() applies enu_ned transform
 *
 * Velocity feed-forward:
 *   PX4 MPC adds the offboard velocity setpoint on top of the position controller
 *   output: desired_vel = pos_controller_output + offboard_velocity.
 *   This reduces tracking lag vs. position-only setpoint_position/local.
 *
 * Acceleration is NOT forwarded (masked out) — keep it simple first.
 *
 * On /mission/stage = "complete", stops forwarding to hand over control
 * to tag_mission_controller for precision landing.
 */

#include <rclcpp/rclcpp.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/string.hpp>

class CmdBridge : public rclcpp::Node {
public:
  explicit CmdBridge(const std::string &cmd_topic = "/position_cmd")
      : Node("cmd_bridge") {
    cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
        cmd_topic, rclcpp::SensorDataQoS(),
        [this](quadrotor_msgs::msg::PositionCommand::ConstSharedPtr cmd) {
          this->cmdCallback(std::move(cmd));
        });

    setpoint_pub_ =
        this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", rclcpp::SensorDataQoS());

    stage_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/mission/stage", 10,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          if (msg->data == "complete" && !stopped_) {
            stopped_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Received /mission/stage='complete' — "
                        "stopping setpoint forwarding for tag handover");
          }
        });

    RCLCPP_INFO(this->get_logger(),
                "Bridging %s -> /mavros/setpoint_raw/local "
                "[ENU, position+velocity+yaw]",
                cmd_topic.c_str());
  }

private:
  void cmdCallback(
      quadrotor_msgs::msg::PositionCommand::ConstSharedPtr cmd) {
    if (stopped_)
      return;

    mavros_msgs::msg::PositionTarget pt;

    pt.header.stamp = cmd->header.stamp;
    pt.header.frame_id = "map";

    // FRAME_LOCAL_NED — despite the name, MAVROS expects ENU input here
    // and auto-converts to NED internally. Do NOT pre-convert.
    pt.coordinate_frame =
        mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

    // type_mask: send position + velocity + acceleration + yaw
    // Ignore yaw_rate only
    pt.type_mask =
        mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

    // ── ENU (NO manual conversion — MAVROS handles it for pos, vel, acc) ──
    pt.position.x = cmd->position.x;
    pt.position.y = cmd->position.y;
    pt.position.z = cmd->position.z;

    pt.velocity.x = cmd->velocity.x;
    pt.velocity.y = cmd->velocity.y;
    pt.velocity.z = cmd->velocity.z;

    pt.acceleration_or_force.x = cmd->acceleration.x;
    pt.acceleration_or_force.y = cmd->acceleration.y;
    pt.acceleration_or_force.z = cmd->acceleration.z;

    // Yaw: ENU convention (0=East, CCW+). MAVROS converts to NED internally.
    pt.yaw = cmd->yaw;

    setpoint_pub_->publish(pt);
  }

  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr
      cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stage_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  bool stopped_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CmdBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
