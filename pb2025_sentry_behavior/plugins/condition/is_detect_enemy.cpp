#include "pb2025_sentry_behavior/plugins/condition/is_detect_enemy.hpp"

#include <string>
#include <chrono>

#include "behaviortree_cpp/bt_factory.h"

namespace pb2025_sentry_behavior
{

IsDetectEnemyCondition::IsDetectEnemyCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList IsDetectEnemyCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "topic_name", "cmd_gimbal",
      "Topic name of gimbal command"),
    BT::InputPort<int>(
      "timeout_ms", 120,
      "If a cmd_gimbal message is received within this timeout, enemy is considered detected")
  };
}

void IsDetectEnemyCondition::initOnce()
{
  if (inited_) {
    return;
  }

  if (!getInput<std::string>("topic_name", topic_name_)) {
    topic_name_ = "cmd_gimbal";
  }

  if (!getInput<int>("timeout_ms", timeout_ms_)) {
    timeout_ms_ = 120;
  }

  try {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  } catch (const std::exception & e) {
    throw BT::RuntimeError(
            std::string("IsDetectEnemy failed to get ROS node from blackboard: ") + e.what());
  }

  sub_ = node_->create_subscription<pb_rm_interfaces::msg::GimbalCmd>(
    topic_name_,
    rclcpp::QoS(10),
    std::bind(&IsDetectEnemyCondition::gimbalCmdCallback, this, std::placeholders::_1));

  inited_ = true;

  RCLCPP_INFO(
    logger_,
    "[IsDetectEnemy] subscribe topic=%s timeout_ms=%d",
    topic_name_.c_str(), timeout_ms_);
}

void IsDetectEnemyCondition::gimbalCmdCallback(
  const pb_rm_interfaces::msg::GimbalCmd::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_msg_time_ = node_->now();
  received_once_ = true;
}

BT::NodeStatus IsDetectEnemyCondition::tick()
{
  initOnce();

  std::lock_guard<std::mutex> lock(mutex_);

  if (!received_once_) {
    return BT::NodeStatus::FAILURE;
  }

  const auto dt = node_->now() - last_msg_time_;
  const auto dt_ns = dt.nanoseconds();
  const auto timeout_ns = static_cast<int64_t>(timeout_ms_) * 1000 * 1000;

  if (dt_ns <= timeout_ns) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace pb2025_sentry_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsDetectEnemyCondition>("IsDetectEnemy");
}
