#pragma once
#include <rclcpp/rclcpp.hpp>

#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"
#include <example_interfaces/msg/u_int8.hpp>

namespace io {

class Publish2Gimbal : public rclcpp::Node {
public:
  Publish2Gimbal();
  void start() {}  // 保持同济风格

  // 绝对角度控制（rad）
  void publish_abs(float yaw, float pitch);

  // 可选：速度控制（rad/s）
  void publish_vel(float yaw_vel, float pitch_vel);

  // 发射：0/1（按 stand_pp_ros2 订阅类型）
  void publish_shoot(uint8_t shoot);

private:
  rclcpp::Publisher<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_pub_;
};

} // namespace io
