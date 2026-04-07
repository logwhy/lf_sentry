#include "pb2025_sentry_behavior/plugins/condition/is_hp_less_than.hpp"

namespace pb2025_sentry_behavior
{

IsHpLessThanCondition::IsHpLessThanCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsHpLessThanCondition::checkHpRatio, this), config)
{
}

BT::NodeStatus IsHpLessThanCondition::checkHpRatio()
{
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available: %s", msg.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  double threshold = 0.0;
  if (!getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger_, "Missing input port [threshold]");
    return BT::NodeStatus::FAILURE;
  }

  if (msg->maximum_hp == 0) {
    RCLCPP_ERROR(logger_, "maximum_hp is zero, cannot calculate hp ratio");
    return BT::NodeStatus::FAILURE;
  }

  const double hp_ratio =
    static_cast<double>(msg->current_hp) / static_cast<double>(msg->maximum_hp);

  RCLCPP_DEBUG(
    logger_,
    "[IsHpLessThan] current_hp=%u, maximum_hp=%u, hp_ratio=%.3f, threshold=%.3f",
    msg->current_hp,
    msg->maximum_hp,
    hp_ratio,
    threshold);

  return (hp_ratio < threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsHpLessThanCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port",
      "{@referee_robotStatus}",
      "RobotStatus port on blackboard"),
    BT::InputPort<double>(
      "threshold",
      "HP ratio threshold (0.0 ~ 1.0)")
  };
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsHpLessThanCondition>("IsHpLessThan");
}