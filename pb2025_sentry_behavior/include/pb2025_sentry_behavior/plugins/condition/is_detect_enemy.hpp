#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"

namespace pb2025_sentry_behavior
{

class IsDetectEnemyCondition : public BT::ConditionNode
{
public:
  IsDetectEnemyCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void initOnce();
  void gimbalCmdCallback(const pb_rm_interfaces::msg::GimbalCmd::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("IsDetectEnemyCondition");

  rclcpp::Subscription<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr sub_;

  std::string topic_name_ = "cmd_gimbal";
  int timeout_ms_ = 120;

  bool inited_ = false;
  bool received_once_ = false;

  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};
  std::mutex mutex_;
};

}  // namespace pb2025_sentry_behavior
