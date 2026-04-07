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

#include "pb2025_sentry_behavior/plugins/action/pub_nav2_goal.hpp"
#include "behaviortree_cpp/basic_types.h"
#include "tf2/LinearMath/Quaternion.h"
#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

PubNav2GoalAction::PubNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

bool PubNav2GoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  // 优先级：goal_pose (PoseStamped) > goal (string)

  // 1) 优先：直接从 Blackboard/上游节点拿 PoseStamped
  if (auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose")) {
    msg = goal_pose.value();
  } else {
    // 2) 退化：从 XML/参数拿字符串并解析为 PoseStamped
    auto goal_str = getInput<std::string>("goal");
    if (!goal_str) {
      RCLCPP_ERROR(logger(), "PubNav2Goal: neither 'goal_pose' nor 'goal' is provided");
      return false;
    }
  try {
    auto parts = BT::splitString(goal_str.value(), ';');

    if (parts.size() != 3 && parts.size() != 4) {
      throw std::runtime_error("expected format 'x;y;yaw' or 'x;y;z;yaw'");
    }

    msg.pose.position.x = BT::convertFromString<double>(parts[0]);
    msg.pose.position.y = BT::convertFromString<double>(parts[1]);

    double yaw = 0.0;
    if (parts.size() == 3) {
      msg.pose.position.z = 0.0;
      yaw = BT::convertFromString<double>(parts[2]);
    } else {
      msg.pose.position.z = BT::convertFromString<double>(parts[2]);
      yaw = BT::convertFromString<double>(parts[3]);
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      logger(), "PubNav2Goal: failed to parse goal string '%s': %s",
      goal_str.value().c_str(), e.what());
    return false;
  }

  }

  // 3) 可选覆盖 frame_id
  if (auto frame_id = getInput<std::string>("frame_id")) {
    if (!frame_id->empty()) {
      msg.header.frame_id = frame_id.value();
    }
  }

  // 4) 补全 Header（发布时 stamp 一般建议用 now()）
  if (msg.header.frame_id.empty()) {
    msg.header.frame_id = "map";
  }
  msg.header.stamp = now();

  return true;
}

BT::PortsList PubNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    // XML/参数侧：用字符串传参（稳定、可控）
    BT::InputPort<std::string>(
      "goal", "0;0;0", "Goal in format 'x;y;yaw' or 'x;y;z;yaw'"),

    // 节点间传递：直接传 PoseStamped（避免二次解析）
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal_pose", "PoseStamped goal from blackboard"),

    // 可选：覆盖 frame_id
    BT::InputPort<std::string>("frame_id", "map", "Frame ID for the goal pose"),
  };

  return providedBasicPorts(additional_ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2GoalAction, "PubNav2Goal");
