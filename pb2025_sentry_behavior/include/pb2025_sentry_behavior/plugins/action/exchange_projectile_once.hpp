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

  // 本次补给区访问是否已经兑换过
  bool exchanged_this_visit_ = false;

  // 是否确认处于补给区访问过程中
  bool was_in_supply_zone_ = false;

  // RFID 离开补给区去抖：不是一帧没检测到就认为离开
  bool leave_candidate_started_ = false;
  rclcpp::Time leave_candidate_start_time_;

  // 兑换冷却：防止 RFID 抖动 / 裁判系统状态延迟导致短时间重复兑换
  bool last_exchange_time_valid_ = false;
  rclcpp::Time last_exchange_time_;

  int exchange_index_ = 0;
  int last_exchange_amount_ = 0;

  std::string topic_name_ = "sentry_cmd";
};

}  // namespace pb2025_sentry_behavior