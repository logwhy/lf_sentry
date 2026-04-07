#include "pb2025_sentry_behavior/plugins/condition/is_dead.hpp"

namespace pb2025_sentry_behavior
{

IsDeadCondition::IsDeadCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDeadCondition::checkRobotDead, this), config)
{
}

BT::NodeStatus IsDeadCondition::checkRobotDead()
{
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available: %s", msg.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const bool is_dead = (msg->current_hp == 0);

  RCLCPP_DEBUG(
    logger_,
    "[IsDead] current_hp=%u, maximum_hp=%u, result=%s",
    msg->current_hp,
    msg->maximum_hp,
    is_dead ? "SUCCESS" : "FAILURE");

  return is_dead ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsDeadCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port",
      "{@referee_robotStatus}",
      "RobotStatus port on blackboard")
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsDeadCondition>("IsDead");
}