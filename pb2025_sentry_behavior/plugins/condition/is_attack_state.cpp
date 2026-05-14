#include "pb2025_sentry_behavior/plugins/condition/is_attack_state.hpp"

#include <cstdint>
#include <functional>

#include "behaviortree_cpp/bt_factory.h"

namespace pb2025_sentry_behavior
{

std::mutex IsAttackStateCondition::shared_mutex_;
bool IsAttackStateCondition::received_once_ = false;
bool IsAttackStateCondition::attack_started_ = false;
rclcpp::Time IsAttackStateCondition::last_enemy_time_{0, 0, RCL_ROS_TIME};
rclcpp::Time IsAttackStateCondition::attack_start_time_{0, 0, RCL_ROS_TIME};
bool IsAttackStateCondition::attack_finished_ = false;

IsAttackStateCondition::IsAttackStateCondition(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList IsAttackStateCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "mode", "keep_attack", "keep_attack / can_leave / enemy_visible / reset"),
    BT::InputPort<std::string>("topic_name", "cmd_gimbal", "Topic name of gimbal command"),
    BT::InputPort<int>("detect_timeout_ms", 500, "Enemy visible timeout in milliseconds"),
    BT::InputPort<double>("no_enemy_timeout_s", 60.0, "No enemy duration before leaving attack"),
    BT::InputPort<double>("min_attack_duration_s", 180.0, "Minimum attack duration before leaving attack")
  };
}

void IsAttackStateCondition::initOnce()
{
  if (inited_) {
    return;
  }

  getInput("topic_name", topic_name_);
  getInput("detect_timeout_ms", detect_timeout_ms_);
  getInput("no_enemy_timeout_s", no_enemy_timeout_s_);
  getInput("min_attack_duration_s", min_attack_duration_s_);

  try {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  } catch (const std::exception & e) {
    throw BT::RuntimeError(
      std::string("IsAttackState failed to get ROS node from blackboard: ") + e.what());
  }

  sub_ = node_->create_subscription<pb_rm_interfaces::msg::GimbalCmd>(
    topic_name_,
    rclcpp::QoS(10),
    std::bind(&IsAttackStateCondition::gimbalCmdCallback, this, std::placeholders::_1));

  inited_ = true;

  RCLCPP_INFO(
    logger_,
    "[IsAttackState] subscribe topic=%s detect_timeout_ms=%d no_enemy_timeout_s=%.1f min_attack_duration_s=%.1f",
    topic_name_.c_str(),
    detect_timeout_ms_,
    no_enemy_timeout_s_,
    min_attack_duration_s_);
}

void IsAttackStateCondition::gimbalCmdCallback(
  const pb_rm_interfaces::msg::GimbalCmd::SharedPtr /*msg*/)
{
  std::lock_guard<std::mutex> lock(shared_mutex_);

  last_enemy_time_ = node_->now();
  received_once_ = true;
}

bool IsAttackStateCondition::isEnemyRecentLocked(const rclcpp::Time & now) const
{
  if (!received_once_) {
    return false;
  }

  const auto dt = now - last_enemy_time_;
  const auto timeout_ns = static_cast<int64_t>(detect_timeout_ms_) * 1000 * 1000;

  return dt.nanoseconds() >= 0 && dt.nanoseconds() <= timeout_ns;
}

bool IsAttackStateCondition::canLeaveAttackLocked(const rclcpp::Time & now) const
{
  if (!attack_started_ || !received_once_) {
    return false;
  }

  if (isEnemyRecentLocked(now)) {
    return false;
  }

  const double no_enemy_s =
    static_cast<double>((now - last_enemy_time_).nanoseconds()) / 1e9;

  const double attack_s =
    static_cast<double>((now - attack_start_time_).nanoseconds()) / 1e9;

  return no_enemy_s >= no_enemy_timeout_s_ && attack_s >= min_attack_duration_s_;
}

BT::NodeStatus IsAttackStateCondition::tick()
{
  initOnce();

  getInput("mode", mode_);
  getInput("detect_timeout_ms", detect_timeout_ms_);
  getInput("no_enemy_timeout_s", no_enemy_timeout_s_);
  getInput("min_attack_duration_s", min_attack_duration_s_);

  std::lock_guard<std::mutex> lock(shared_mutex_);

  const auto now = node_->now();
  const bool enemy_recent = isEnemyRecentLocked(now);
  const bool can_leave = canLeaveAttackLocked(now);

  if (mode_ == "reset") {
    received_once_ = false;
    attack_started_ = false;
    attack_finished_ = false;
    last_enemy_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    attack_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    return BT::NodeStatus::SUCCESS;
  }

  if (mode_ == "enemy_visible") {
    if (!enemy_recent) {
      return BT::NodeStatus::FAILURE;
    }

    if (!attack_started_ || attack_finished_) {
      attack_started_ = true;
      attack_finished_ = false;
      attack_start_time_ = now;
    }

    return BT::NodeStatus::SUCCESS;
  }

  if (mode_ == "can_leave") {
    if (can_leave) {
      attack_finished_ = true;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  if (mode_ == "keep_attack") {
    return (attack_started_ && !can_leave) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(logger_, "Unknown mode: %s", mode_.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace pb2025_sentry_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsAttackStateCondition>("IsAttackState");
}
