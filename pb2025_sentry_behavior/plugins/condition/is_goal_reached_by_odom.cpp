#include "pb2025_sentry_behavior/plugins/condition/is_goal_reached_by_odom.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp/basic_types.h"

namespace pb2025_sentry_behavior
{

IsGoalReachedByOdom::IsGoalReachedByOdom(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<nav_msgs::msg::Odometry>(name, conf, params)
{
}

BT::NodeStatus IsGoalReachedByOdom::onTick(
  const std::shared_ptr<nav_msgs::msg::Odometry> & last_msg)
{
  if (!last_msg) {
    RCLCPP_WARN(logger(), "IsGoalReachedByOdom: no odometry received yet");
    return BT::NodeStatus::FAILURE;
  }

  auto goal_str = getInput<std::string>("goal");
  if (!goal_str) {
    throw BT::RuntimeError("IsGoalReachedByOdom: missing input [goal]");
  }

  const auto parts = BT::splitString(goal_str.value(), ';');
  if (parts.size() != 3 && parts.size() != 4) {
    throw BT::RuntimeError(
      "IsGoalReachedByOdom: invalid goal format, expected x;y;yaw or x;y;z;yaw");
  }

  const double goal_x = BT::convertFromString<double>(parts[0]);
  const double goal_y = BT::convertFromString<double>(parts[1]);

  const double pos_tolerance =
    getInput<double>("pos_tolerance").value_or(0.50);

  const double x = last_msg->pose.pose.position.x;
  const double y = last_msg->pose.pose.position.y;

  const double dx = x - goal_x;
  const double dy = y - goal_y;
  const double dist = std::hypot(dx, dy);

  RCLCPP_INFO(
    logger(),
    "IsGoalReachedByOdom: curr=(%.3f, %.3f), goal=(%.3f, %.3f), dist=%.3f, tol=%.3f",
    x, y, goal_x, goal_y, dist, pos_tolerance);

  if (dist <= pos_tolerance) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT::PortsList IsGoalReachedByOdom::providedPorts()
{
  BT::PortsList ports = {
    BT::InputPort<std::string>("goal", "0;0;0", "Goal format: x;y;yaw or x;y;z;yaw"),
    BT::InputPort<double>("pos_tolerance", 0.50, "Position tolerance in meters"),
  };

  return providedBasicPorts(ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::IsGoalReachedByOdom, "IsGoalReachedByOdom");

//话题/red_standard_robot1/navigate_to_pose/_action/status也可以检验到点，status=4就行