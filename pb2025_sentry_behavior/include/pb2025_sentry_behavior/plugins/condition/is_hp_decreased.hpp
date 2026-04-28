#pragma once

#include <deque>
#include <optional>
#include <string>
#include <utility>

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsHpDecreased : public BT::ConditionNode
{
public:
  IsHpDecreased(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void initOnce();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("IsHpDecreased");

  bool inited_ = false;

  // 保存最近一段时间内的血量记录：time, hp
  std::deque<std::pair<rclcpp::Time, int>> hp_history_;

  // 触发后保持 SUCCESS 到这个时间
  std::optional<rclcpp::Time> hold_until_;
};

}  // namespace pb2025_sentry_behavior