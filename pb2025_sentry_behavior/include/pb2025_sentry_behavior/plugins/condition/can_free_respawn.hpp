#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CAN_FREE_RESPAWN_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CAN_FREE_RESPAWN_HPP_

#include <functional>

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/sentry_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class CanFreeRespawnCondition : public BT::SimpleConditionNode
{
public:
  CanFreeRespawnCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkCanFreeRespawn();

  rclcpp::Logger logger_ = rclcpp::get_logger("CanFreeRespawnCondition");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__CAN_FREE_RESPAWN_HPP_