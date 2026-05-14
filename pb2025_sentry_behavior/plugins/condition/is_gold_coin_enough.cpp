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

#include "pb2025_sentry_behavior/plugins/condition/is_gold_coin_enough.hpp"

#include <functional>

#include "behaviortree_cpp/bt_factory.h"

namespace pb2025_sentry_behavior
{

IsGoldCoinEnoughCondition::IsGoldCoinEnoughCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsGoldCoinEnoughCondition::checkGoldCoin, this), config)
{
}

BT::NodeStatus IsGoldCoinEnoughCondition::checkGoldCoin()
{
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  int min_gold = 150;
  getInput("min_gold", min_gold);

  const int remaining_gold_coin = static_cast<int>(msg->remaining_gold_coin);
  return remaining_gold_coin >= min_gold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsGoldCoinEnoughCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<int>("min_gold", 150, "Minimum remaining gold coin")};
}

}  // namespace pb2025_sentry_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsGoldCoinEnoughCondition>(
    "IsGoldCoinEnough");
}
