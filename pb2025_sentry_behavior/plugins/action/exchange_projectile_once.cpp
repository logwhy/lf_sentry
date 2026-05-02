#include "pb2025_sentry_behavior/plugins/action/exchange_projectile_once.hpp"

#include <algorithm>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace pb2025_sentry_behavior
{

ExchangeProjectileOnceAction::ExchangeProjectileOnceAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList ExchangeProjectileOnceAction::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RfidStatus>(
      "rfid_key_port",
      "{@referee_rfidStatus}",
      "RfidStatus port on blackboard"),

    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "robot_status_key_port",
      "{@referee_robotStatus}",
      "RobotStatus port on blackboard"),

    BT::InputPort<bool>(
      "friendly_supply_zone_exchange",
      true,
      "Use friendly supply zone exchange flag"),

    BT::InputPort<bool>(
      "friendly_supply_zone_non_exchange",
      true,
      "Use friendly supply zone non-exchange flag"),

    BT::InputPort<int>(
      "ammo_threshold",
      50,
      "Only exchange when projectile allowance is lower than this value"),

    BT::InputPort<int>(
      "base_exchange",
      150,
      "First exchange projectile amount"),

    BT::InputPort<int>(
      "increment_step",
      1,
      "Increase exchange amount by this value every new supply-zone visit"),

    BT::InputPort<int>(
      "max_exchange",
      500,
      "Maximum projectile exchange value clamp"),

    BT::InputPort<std::string>(
      "topic_name",
      "sentry_cmd",
      "Topic name of SentryCmd")
  };
}

void ExchangeProjectileOnceAction::initOnce()
{
  if (inited_) {
    return;
  }

  try {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  } catch (const std::exception & e) {
    throw BT::RuntimeError(
      std::string("ExchangeProjectileOnce failed to get ROS node from blackboard: ") + e.what());
  }

  getInput("topic_name", topic_name_);
  if (topic_name_.empty()) {
    topic_name_ = "sentry_cmd";
  }

  pub_ = node_->create_publisher<pb_rm_interfaces::msg::SentryCmd>(
    topic_name_,
    rclcpp::QoS(10));

  inited_ = true;

  RCLCPP_INFO(
    logger_,
    "[ExchangeProjectileOnce] publish topic=%s",
    topic_name_.c_str());
}

bool ExchangeProjectileOnceAction::isInSupplyZone(
  const pb_rm_interfaces::msg::RfidStatus & rfid_msg) const
{
  bool use_exchange = true;
  bool use_non_exchange = true;

  getInput("friendly_supply_zone_exchange", use_exchange);
  getInput("friendly_supply_zone_non_exchange", use_non_exchange);

  const bool in_exchange_zone =
    use_exchange &&
    rfid_msg.friendly_supply_zone_exchange == rfid_msg.DETECTED;

  const bool in_non_exchange_zone =
    use_non_exchange &&
    rfid_msg.friendly_supply_zone_non_exchange == rfid_msg.DETECTED;

  return in_exchange_zone || in_non_exchange_zone;
}

void ExchangeProjectileOnceAction::publishExchangeCommand(int exchange_amount)
{
  pb_rm_interfaces::msg::SentryCmd msg;

  msg.confirm_respawn = false;
  msg.confirm_pay_respawn = false;
  msg.projectile_allowance_to_exchange = static_cast<uint16_t>(std::max(0, exchange_amount));
  msg.remote_projectile_request_count = 0;
  msg.remote_hp_request_count = 0;
  msg.posture_command = 0;
  msg.activate_rune = false;

  pub_->publish(msg);
}

BT::NodeStatus ExchangeProjectileOnceAction::tick()
{
  initOnce();

  auto rfid_msg = getInput<pb_rm_interfaces::msg::RfidStatus>("rfid_key_port");
  if (!rfid_msg) {
    RCLCPP_WARN(logger_, "[ExchangeProjectileOnce] missing RfidStatus: %s", rfid_msg.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto robot_status_msg =
    getInput<pb_rm_interfaces::msg::RobotStatus>("robot_status_key_port");
  if (!robot_status_msg) {
    RCLCPP_WARN(
      logger_,
      "[ExchangeProjectileOnce] missing RobotStatus: %s",
      robot_status_msg.error().c_str());
    return BT::NodeStatus::FAILURE;
  }

  int ammo_threshold = 50;
  int base_exchange = 150;
  int increment_step = 1;
  int max_exchange = 500;

  getInput("ammo_threshold", ammo_threshold);
  getInput("base_exchange", base_exchange);
  getInput("increment_step", increment_step);
  getInput("max_exchange", max_exchange);

  const bool in_supply_zone = isInSupplyZone(rfid_msg.value());
  const int current_ammo =
    static_cast<int>(robot_status_msg.value().projectile_allowance_17mm);

  // 不在补给区：认为本次补给区访问结束，允许下一次进入补给区时再次兑换
  if (!in_supply_zone) {
    if (was_in_supply_zone_) {
      RCLCPP_INFO(
        logger_,
        "[ExchangeProjectileOnce] leave supply zone, next visit can exchange again");
    }

    was_in_supply_zone_ = false;
    exchanged_this_visit_ = false;
    return BT::NodeStatus::FAILURE;
  }

  was_in_supply_zone_ = true;

  // 在补给区，但弹丸不少于阈值：不兑换，但保持“本次访问”的状态
  if (current_ammo >= ammo_threshold) {
    return BT::NodeStatus::FAILURE;
  }

  // 在补给区，弹丸少于阈值，并且本次访问还没有兑换过：只兑换一次
  if (!exchanged_this_visit_) {
    int exchange_amount = base_exchange + exchange_index_ * increment_step;
    exchange_amount = std::clamp(exchange_amount, 0, max_exchange);

    publishExchangeCommand(exchange_amount);

    last_exchange_amount_ = exchange_amount;
    exchanged_this_visit_ = true;
    exchange_index_++;

    RCLCPP_WARN(
      logger_,
      "[ExchangeProjectileOnce] exchange once this visit: ammo=%d < %d, amount=%d, next_amount=%d",
      current_ammo,
      ammo_threshold,
      exchange_amount,
      std::clamp(base_exchange + exchange_index_ * increment_step, 0, max_exchange));

    return BT::NodeStatus::SUCCESS;
  }

  // 在同一次补给区访问期间，不再重复兑换，兑弹数量也不改变
  RCLCPP_DEBUG(
    logger_,
    "[ExchangeProjectileOnce] already exchanged this visit, keep amount=%d, no republish",
    last_exchange_amount_);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace pb2025_sentry_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::ExchangeProjectileOnceAction>(
    "ExchangeProjectileOnce");
}
