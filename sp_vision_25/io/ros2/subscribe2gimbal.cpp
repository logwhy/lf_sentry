#include "io/ros2/subscribe2gimbal.hpp"

namespace io {

Subscribe2Gimbal::Subscribe2Gimbal()
: rclcpp::Node("subscribe2gimbal")
{
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "serial/imu", rclcpp::QoS(10),
    std::bind(&Subscribe2Gimbal::imu_callback, this, std::placeholders::_1));

  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "serial/gimbal_joint_state", rclcpp::QoS(10),
    std::bind(&Subscribe2Gimbal::joint_callback, this, std::placeholders::_1));
}

void Subscribe2Gimbal::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_q_.push(*msg);
}

void Subscribe2Gimbal::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  GimbalJointFeedback fb;
  fb.stamp = msg->header.stamp;

  // 鲁棒解析：关节名包含 yaw/pitch 即可
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    const auto & n = msg->name[i];
    if (n.find("yaw") != std::string::npos)   fb.yaw = static_cast<float>(msg->position[i]);
    if (n.find("pitch") != std::string::npos) fb.pitch = static_cast<float>(msg->position[i]);
  }

  fb.valid = true;
  joint_q_.push(fb);
}

std::optional<sensor_msgs::msg::Imu> Subscribe2Gimbal::subscribe_imu()
{
  sensor_msgs::msg::Imu out;
  if (imu_q_.try_pop(out)) return out;
  return std::nullopt;
}

std::optional<GimbalJointFeedback> Subscribe2Gimbal::subscribe_gimbal_joint()
{
  GimbalJointFeedback out;
  if (joint_q_.try_pop(out)) return out;
  return std::nullopt;
}

} // namespace io
