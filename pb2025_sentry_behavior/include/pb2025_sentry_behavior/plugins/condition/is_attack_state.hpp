#pragma once

#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsAttackStateCondition : public BT::ConditionNode
{
public:
  IsAttackStateCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void initOnce();
  void gimbalCmdCallback(const pb_rm_interfaces::msg::GimbalCmd::SharedPtr msg);
  bool canLeaveAttackLocked(const rclcpp::Time & now) const;
  bool isEnemyRecentLocked(const rclcpp::Time & now) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr sub_;
  rclcpp::Logger logger_ = rclcpp::get_logger("IsAttackStateCondition");

  std::mutex mutex_;

  bool inited_ = false;
  bool received_once_ = false;
  bool attack_started_ = false;

  std::string topic_name_ = "cmd_gimbal";
  std::string mode_ = "keep_attack";

  int detect_timeout_ms_ = 500;
  double no_enemy_timeout_s_ = 60.0;
  double min_attack_duration_s_ = 180.0;

  rclcpp::Time last_enemy_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time attack_start_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace pb2025_sentry_behavior
