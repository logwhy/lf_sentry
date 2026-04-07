#include "pb2025_sentry_behavior/plugins/condition/can_free_respawn.hpp"

namespace pb2025_sentry_behavior
{

CanFreeRespawnCondition::CanFreeRespawnCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&CanFreeRespawnCondition::checkCanFreeRespawn, this), config)
{
}

BT::NodeStatus CanFreeRespawnCondition::checkCanFreeRespawn()
{
  auto msg = getInput<pb_rm_interfaces::msg::SentryInfo>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "SentryInfo message is not available: %s", msg.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG(
    logger_,
    "[CanFreeRespawn] can_free_respawn=%s, can_pay_respawn=%s, respawn_gold_cost=%u",
    msg->can_free_respawn ? "true" : "false",
    msg->can_pay_respawn ? "true" : "false",
    msg->respawn_gold_cost);

  return msg->can_free_respawn ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList CanFreeRespawnCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::SentryInfo>(
      "key_port",
      "{@referee_sentryInfo}",
      "SentryInfo port on blackboard")
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::CanFreeRespawnCondition>("CanFreeRespawn");
}