#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DEAD_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DEAD_HPP_

#include <functional>

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsDeadCondition : public BT::SimpleConditionNode
{
public:
  IsDeadCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkRobotDead();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsDeadCondition");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DEAD_HPP_