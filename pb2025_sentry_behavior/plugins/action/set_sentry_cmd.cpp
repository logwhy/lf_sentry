#include "pb2025_sentry_behavior/plugins/action/set_sentry_cmd.hpp"

namespace pb2025_sentry_behavior
{

SetSentryCmdAction::SetSentryCmdAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicPubNode<pb_rm_interfaces::msg::SentryCmd>(name, conf, params)
{
}

BT::PortsList SetSentryCmdAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<bool>("confirm_respawn", false, "Confirm free respawn"),
    BT::InputPort<bool>("confirm_pay_respawn", false, "Confirm paid respawn"),
    BT::InputPort<int>("projectile_allowance_to_exchange", 0, "Projectile allowance to exchange"),
    BT::InputPort<int>("remote_projectile_request_count", 0, "Remote projectile exchange request count"),
    BT::InputPort<int>("remote_hp_request_count", 0, "Remote hp exchange request count"),
    BT::InputPort<int>("posture_command", 0, "Posture command (1 attack, 2 defense, 3 move)"),
    BT::InputPort<bool>("activate_rune", false, "Activate rune"),
  };

  return providedBasicPorts(additional_ports);
}

bool SetSentryCmdAction::setMessage(pb_rm_interfaces::msg::SentryCmd & msg)
{
  bool confirm_respawn = false;
  bool confirm_pay_respawn = false;
  bool activate_rune = false;
  int projectile_allowance_to_exchange = 0;
  int remote_projectile_request_count = 0;
  int remote_hp_request_count = 0;
  int posture_command = 0;

  getInput("confirm_respawn", confirm_respawn);
  getInput("confirm_pay_respawn", confirm_pay_respawn);
  getInput("projectile_allowance_to_exchange", projectile_allowance_to_exchange);
  getInput("remote_projectile_request_count", remote_projectile_request_count);
  getInput("remote_hp_request_count", remote_hp_request_count);
  getInput("posture_command", posture_command);
  getInput("activate_rune", activate_rune);

  // 每次 tick 都构造一条全新的命令，避免旧值残留
  msg.confirm_respawn = confirm_respawn;
  msg.confirm_pay_respawn = confirm_pay_respawn;
  msg.projectile_allowance_to_exchange =
    static_cast<uint16_t>(std::max(0, projectile_allowance_to_exchange));
  msg.remote_projectile_request_count =
    static_cast<uint8_t>(std::max(0, remote_projectile_request_count));
  msg.remote_hp_request_count =
    static_cast<uint8_t>(std::max(0, remote_hp_request_count));
  msg.posture_command =
    static_cast<uint8_t>(std::max(0, posture_command));
  msg.activate_rune = activate_rune;

  RCLCPP_DEBUG(
    logger(),
    "[SetSentryCmd] confirm_respawn=%s, confirm_pay_respawn=%s, "
    "projectile_allowance_to_exchange=%u, remote_projectile_request_count=%u, "
    "remote_hp_request_count=%u, posture_command=%u, activate_rune=%s",
    msg.confirm_respawn ? "true" : "false",
    msg.confirm_pay_respawn ? "true" : "false",
    msg.projectile_allowance_to_exchange,
    msg.remote_projectile_request_count,
    msg.remote_hp_request_count,
    msg.posture_command,
    msg.activate_rune ? "true" : "false");

  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"

CreateRosNodePlugin(pb2025_sentry_behavior::SetSentryCmdAction, "SetSentryCmd");