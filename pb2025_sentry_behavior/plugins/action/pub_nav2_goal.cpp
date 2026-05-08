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

#include <cmath>
#include <stdexcept>

#include "behaviortree_cpp/basic_types.h"
#include "pb2025_sentry_behavior/custom_types.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace pb2025_sentry_behavior
{

PubNav2GoalAction::PubNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

namespace
{
bool fillGoalFromString(
  const std::string & goal_str, geometry_msgs::msg::PoseStamped & msg, rclcpp::Logger logger)
{
  try {
    auto parts = BT::splitString(goal_str, ';');

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
      logger, "PubNav2Goal: failed to parse goal string '%s': %s", goal_str.c_str(), e.what());
    return false;
  }

  return true;
}

void fillGoalFromMapCommand(
  const geometry_msgs::msg::Point & command, const std::string & team, double red_origin_x,
  double red_origin_y, double blue_origin_x, double blue_origin_y, double offset_x,
  double offset_y, double yaw, rclcpp::Logger logger,
  geometry_msgs::msg::PoseStamped & msg)
{
  double x = command.x - red_origin_x;
  double y = command.y - red_origin_y;

  if (team == "blue") {
    x = blue_origin_x - command.x;
    y = blue_origin_y - command.y;
  } else if (team != "red") {
    RCLCPP_WARN(logger, "PubNav2Goal: unknown map_command_team '%s', using red", team.c_str());
  }

  msg.pose.position.x = x + offset_x;
  msg.pose.position.y = y + offset_y;
  msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
}
}  // namespace

bool PubNav2GoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  if (auto goal_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose")) {
    msg = goal_pose.value();
  } else if (auto map_command = getInput<geometry_msgs::msg::Point>("map_command");
             map_command &&
             (std::abs(map_command.value().x) > 1e-6 ||
              std::abs(map_command.value().y) > 1e-6)) {
    const double offset_x = getInput<double>("map_command_offset_x").value_or(0.0);
    const double offset_y = getInput<double>("map_command_offset_y").value_or(0.0);
    const double yaw = getInput<double>("map_command_yaw").value_or(0.0);
    const std::string team = getInput<std::string>("map_command_team").value_or("red");
    const double red_origin_x = getInput<double>("map_command_red_origin_x").value_or(2.7);
    const double red_origin_y = getInput<double>("map_command_red_origin_y").value_or(9.9);
    const double blue_origin_x = getInput<double>("map_command_blue_origin_x").value_or(26.0);
    const double blue_origin_y = getInput<double>("map_command_blue_origin_y").value_or(6.0);
    fillGoalFromMapCommand(
      map_command.value(), team, red_origin_x, red_origin_y, blue_origin_x, blue_origin_y,
      offset_x, offset_y, yaw, logger(), msg);
  } else {
    auto goal_str = getInput<std::string>("goal");
    if (!goal_str) {
      RCLCPP_ERROR(logger(), "PubNav2Goal: no valid 'goal_pose', 'map_command', or 'goal'");
      return false;
    }

    if (!fillGoalFromString(goal_str.value(), msg, logger())) {
      return false;
    }
  }

  if (auto frame_id = getInput<std::string>("frame_id")) {
    if (!frame_id->empty()) {
      msg.header.frame_id = frame_id.value();
    }
  }

  if (msg.header.frame_id.empty()) {
    msg.header.frame_id = "map";
  }
  msg.header.stamp = now();

  return true;
}

BT::PortsList PubNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<std::string>(
      "goal", "0;0;0", "Goal in format 'x;y;yaw' or 'x;y;z;yaw'"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal_pose", "PoseStamped goal from blackboard"),
    BT::InputPort<geometry_msgs::msg::Point>("map_command", "Map command point from blackboard"),
    BT::InputPort<std::string>("map_command_team", "red", "red or blue map command transform"),
    BT::InputPort<double>("map_command_red_origin_x", 2.7, "Red origin X in field coordinates"),
    BT::InputPort<double>("map_command_red_origin_y", 9.9, "Red origin Y in field coordinates"),
    BT::InputPort<double>("map_command_blue_origin_x", 26.0, "Blue origin X in field coordinates"),
    BT::InputPort<double>("map_command_blue_origin_y", 6.0, "Blue origin Y in field coordinates"),
    BT::InputPort<double>("map_command_offset_x", 0.0, "Offset added to map command X"),
    BT::InputPort<double>("map_command_offset_y", 0.0, "Offset added to map command Y"),
    BT::InputPort<double>("map_command_yaw", 0.0, "Yaw used for map command goal"),
    BT::InputPort<std::string>("frame_id", "map", "Frame ID for the goal pose"),
  };

  return providedBasicPorts(additional_ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2GoalAction, "PubNav2Goal");
