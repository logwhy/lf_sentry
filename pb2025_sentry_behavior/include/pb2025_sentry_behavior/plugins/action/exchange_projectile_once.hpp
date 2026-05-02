#pragma once

#include <cstdint>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "pb_rm_interfaces/msg/sentry_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class ExchangeProjectileOnceAction : public BT::SyncActionNode
{
public:
  ExchangeProjectileOnceAction(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void initOnce();
  bool isInSupplyZone(const pb_rm_interfaces::msg::RfidStatus & rfid_msg) const;
  void publishExchangeCommand(int exchange_amount);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<pb_rm_interfaces::msg::SentryCmd>::SharedPtr pub_;
  rclcpp::Logger logger_ = rclcpp::get_logger("ExchangeProjectileOnceAction");

  bool inited_ = false;

  bool exchanged_this_visit_ = false;
  bool was_in_supply_zone_ = false;

  int exchange_index_ = 0;
  int last_exchange_amount_ = 0;

  std::string topic_name_ = "sentry_cmd";
};

}  // namespace pb2025_sentry_behavior
