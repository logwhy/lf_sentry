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
      "RobotStatus port on blackboard"),

    BT::InputPort<double>(
      "window_s",
      3.0,
      "Time window for HP drop detection, in seconds"),

    BT::InputPort<int>(
      "drop_threshold",
      40,
      "Trigger if HP drop in window is greater than this value"),

    BT::InputPort<double>(
      "hold_s",
      10.0,
      "Keep SUCCESS after triggered, in seconds")
  };
}

void IsHpDecreased::initOnce()
{
  if (inited_) {
    return;
  }

  try {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  } catch (const std::exception & e) {
    throw BT::RuntimeError(
      std::string("IsHpDecreased failed to get ROS node from blackboard: ") + e.what());
  }

  inited_ = true;
}

BT::NodeStatus IsHpDecreased::tick()
{
  initOnce();

  auto robot_status = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!robot_status) {
    throw BT::RuntimeError(
      "IsHpDecreased: missing required input [key_port]: ",
      robot_status.error());
  }

  double window_s = 3.0;
  int drop_threshold = 40;
  double hold_s = 10.0;

  getInput("window_s", window_s);
  getInput("drop_threshold", drop_threshold);
  getInput("hold_s", hold_s);

  const auto now = node_->now();
  const int current_hp = static_cast<int>(robot_status.value().current_hp);

  // 如果还处于保持时间内，继续返回 SUCCESS
  if (hold_until_.has_value() && now < hold_until_.value()) {
    return BT::NodeStatus::SUCCESS;
  }

  // 超过保持时间后清除
  if (hold_until_.has_value() && now >= hold_until_.value()) {
    hold_until_.reset();
  }

  // 记录当前血量
  hp_history_.emplace_back(now, current_hp);

  // 删除超过 window_s 的旧记录
  while (!hp_history_.empty()) {
    const double age_s =
      static_cast<double>((now - hp_history_.front().first).nanoseconds()) / 1e9;

    if (age_s <= window_s) {
      break;
    }

    hp_history_.pop_front();
  }

  if (hp_history_.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  // 取窗口内最高血量，与当前血量比较
  int max_hp_in_window = current_hp;
  for (const auto & item : hp_history_) {
    if (item.second > max_hp_in_window) {
      max_hp_in_window = item.second;
    }
  }

  const int hp_drop = max_hp_in_window - current_hp;

  // 注意：你说的是“大于40”，所以这里用 >，不是 >=
  if (hp_drop > drop_threshold) {
    hold_until_ = now + rclcpp::Duration::from_seconds(hold_s);

    RCLCPP_WARN(
      logger_,
      "[IsHpDecreased] Triggered: hp_drop=%d > threshold=%d, hold %.1f s",
      hp_drop,
      drop_threshold,
      hold_s);

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