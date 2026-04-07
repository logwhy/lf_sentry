#pragma once
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "tools/thread_safe_queue.hpp"

namespace io {

struct GimbalJointFeedback {
  rclcpp::Time stamp;
  float yaw = 0.f;
  float pitch = 0.f;
  bool valid = false;
};

class Subscribe2Gimbal : public rclcpp::Node {
public:
  Subscribe2Gimbal();
  void start() {} // 同济风格

  std::optional<sensor_msgs::msg::Imu> subscribe_imu();
  std::optional<GimbalJointFeedback> subscribe_gimbal_joint();

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  tools::ThreadSafeQueue<sensor_msgs::msg::Imu> imu_q_{1000};
  tools::ThreadSafeQueue<GimbalJointFeedback> joint_q_{1000};
};

} // namespace io
