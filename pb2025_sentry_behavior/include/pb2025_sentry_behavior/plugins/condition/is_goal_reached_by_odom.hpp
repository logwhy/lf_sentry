#pragma once

#include <string>

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace pb2025_sentry_behavior
{

class IsGoalReachedByOdom : public BT::RosTopicSubNode<nav_msgs::msg::Odometry>
{
public:
  IsGoalReachedByOdom(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onTick(const std::shared_ptr<nav_msgs::msg::Odometry> & last_msg) override;
};

}  // namespace pb2025_sentry_behavior