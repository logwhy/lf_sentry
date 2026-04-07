#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_HP_LESS_THAN_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_HP_LESS_THAN_HPP_

#include <functional>

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsHpLessThanCondition : public BT::SimpleConditionNode
{
public:
  IsHpLessThanCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkHpRatio();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsHpLessThanCondition");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_HP_LESS_THAN_HPP_