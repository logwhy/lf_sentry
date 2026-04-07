// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb2025_sentry_behavior/plugins/action/send_nav2_goal.hpp"

#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

SendNav2GoalAction::SendNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
}

bool SendNav2GoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  // 优先级：goal_pose (PoseStamped) > goal (string)

  // 1) 优先：直接从 Blackboard/上游节点拿 PoseStamped
  if (auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose")) {
    goal.pose = goal_pose.value();
  } else {
    // 2) 退化：从 XML/参数拿字符串并解析为 PoseStamped
    auto goal_str = getInput<std::string>("goal");
    if (!goal_str) {
      RCLCPP_ERROR(logger(), "SendNav2Goal: neither 'goal_pose' nor 'goal' is provided");
      return false;
    }

    try {
      goal.pose = BT::convertFromString<geometry_msgs::msg::PoseStamped>(goal_str.value());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger(), "SendNav2Goal: failed to parse goal string '%s': %s",
        goal_str.value().c_str(), e.what());
      return false;
    }
  }

  // 3) 可选覆盖 frame_id（方便你在 XML 里改 map/odom 等）
  if (auto frame_id = getInput<std::string>("frame_id")) {
    if (!frame_id->empty()) {
      goal.pose.header.frame_id = frame_id.value();
    }
  }

  // 4) 补全 Header
  if (goal.pose.header.frame_id.empty()) {
    goal.pose.header.frame_id = "map";
  }
  goal.pose.header.stamp = now();

  return true;
}

BT::NodeStatus SendNav2GoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "Navigation succeeded!");
      return BT::NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "Navigation aborted by server");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger(), "Navigation canceled");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "Unknown navigation result code: %d", static_cast<int>(wr.code));
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SendNav2GoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_DEBUG(logger(), "Distance remaining: %f", feedback->distance_remaining);
  return BT::NodeStatus::RUNNING;
}

void SendNav2GoalAction::onHalt() { RCLCPP_INFO(logger(), "SendNav2GoalAction has been halted."); }

BT::NodeStatus SendNav2GoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "SendNav2GoalAction failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

BT::PortsList SendNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    // XML/参数侧：用字符串传参（稳定、可控）
    BT::InputPort<std::string>(
      "goal", "0;0;0", "Goal in format 'x;y;yaw' or 'x;y;z;yaw'"),

    // 节点间传递：直接传 PoseStamped（避免二次解析）
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal_pose", "PoseStamped goal from blackboard"),

    // 可选：覆盖 frame_id
    BT::InputPort<std::string>("frame_id", "map", "Frame ID for goal pose"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::SendNav2GoalAction, "SendNav2Goal");
