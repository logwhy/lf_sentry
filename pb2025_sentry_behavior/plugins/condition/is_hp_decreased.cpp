#include "pb2025_sentry_behavior/plugins/condition/is_hp_decreased.hpp"

#include <string>

namespace pb2025_sentry_behavior
{

IsHpDecreased::IsHpDecreased(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList IsHpDecreased::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port",
      "{@referee_robotStatus}",
      "RobotStatus port on blackboard")
  };
}

BT::NodeStatus IsHpDecreased::tick()
{
  auto robot_status = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!robot_status) {
    throw BT::RuntimeError(
      "IsHpDecreased: missing required input [key_port]: ",
      robot_status.error());
  }

  const int current_hp = static_cast<int>(robot_status.value().current_hp);

  // 第一次进入时，只记录，不触发“掉血”
  if (!last_hp_.has_value()) {
    last_hp_ = current_hp;
    return BT::NodeStatus::FAILURE;
  }

 static int hold = 0;

  const bool hp_decreased = current_hp < last_hp_.value() - 20;

  // 每次 tick 后更新缓存
  last_hp_ = current_hp;

  if (hp_decreased) {
    hold = 20;   // 这里改数字：掉血后连续保持 SUCCESS 的 tick 数
    return BT::NodeStatus::SUCCESS;
  }

  if (hold > 0) {
    --hold;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsHpDecreased>("IsHpDecreased");
}
