#pragma once

#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{

class CalculateChaseGoal : public BT::SyncActionNode
{
public:
  CalculateChaseGoal(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void initOnce();
  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

private:
  bool inited_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("CalculateChaseGoal")};

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  geometry_msgs::msg::PointStamped last_target_;
  rclcpp::Time last_recv_time_{0, 0, RCL_ROS_TIME};
  bool has_target_{false};

  std::string topic_name_;
  std::string source_frame_;
  std::string world_frame_;
  std::string robot_base_frame_;

  double target_timeout_{0.35};
  double chase_distance_{1.5};
  double stop_tolerance_{0.15};

  // 相机系筛选
  double z_min_{0.8};
  double z_max_{6.0};
  double x_abs_max_{2.5};
  double y_abs_max_{1.5};

  // 目标点允许区域（world/odom）
  double world_x_min_{-100.0};
  double world_x_max_{100.0};
  double world_y_min_{-100.0};
  double world_y_max_{100.0};

  // 追击目标点允许区域（world/odom）
  double goal_x_min_{-100.0};
  double goal_x_max_{100.0};
  double goal_y_min_{-100.0};
  double goal_y_max_{100.0};

  double transform_tolerance_{0.2};
};

}  // namespace pb2025_sentry_behavior
