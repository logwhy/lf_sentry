#pragma once

#include <optional>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/robot_status.hpp"

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
  std::optional<int> last_hp_;
};

}  // namespace pb2025_sentry_behavior
