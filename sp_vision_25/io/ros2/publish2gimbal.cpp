#include "io/ros2/publish2gimbal.hpp"

namespace io {

Publish2Gimbal::Publish2Gimbal()
: rclcpp::Node("publish2gimbal")
{
  gimbal_pub_ = this->create_publisher<pb_rm_interfaces::msg::GimbalCmd>("cmd_gimbal", rclcpp::QoS(10));
  shoot_pub_  = this->create_publisher<example_interfaces::msg::UInt8>("cmd_shoot", rclcpp::QoS(10));
}

void Publish2Gimbal::publish_abs(float yaw, float pitch)
{
  pb_rm_interfaces::msg::GimbalCmd msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  msg.yaw_type   = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  msg.pitch_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;

  msg.position.yaw = yaw;
  msg.position.pitch = pitch;

  // 给默认范围（即便驱动不用也不影响）
  msg.position.yaw_min_range = -3.14159f;
  msg.position.yaw_max_range =  3.14159f;
  msg.position.pitch_min_range = -1.57f;
  msg.position.pitch_max_range =  1.57f;

  gimbal_pub_->publish(msg);
}

void Publish2Gimbal::publish_vel(float yaw_vel, float pitch_vel)
{
  pb_rm_interfaces::msg::GimbalCmd msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  msg.yaw_type   = pb_rm_interfaces::msg::GimbalCmd::VELOCITY;
  msg.pitch_type = pb_rm_interfaces::msg::GimbalCmd::VELOCITY;

  msg.velocity.yaw = yaw_vel;
  msg.velocity.pitch = pitch_vel;

  gimbal_pub_->publish(msg);
}

void Publish2Gimbal::publish_shoot(uint8_t shoot)
{
  example_interfaces::msg::UInt8 s;
  s.data = shoot;      // 0/1
  shoot_pub_->publish(s);
}

} // namespace io
