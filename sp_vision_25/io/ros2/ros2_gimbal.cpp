#include "io/ros2/ros2_gimbal.hpp"
#include <rclcpp/rclcpp.hpp>

namespace io {

ROS2Gimbal::ROS2Gimbal()
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  pub_ = std::make_shared<Publish2Gimbal>();
  sub_ = std::make_shared<Subscribe2Gimbal>();

  spin_thread_ = std::make_unique<std::thread>([this]() {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(pub_);
    exec.add_node(sub_);
    exec.spin();
  });
}

ROS2Gimbal::~ROS2Gimbal()
{
  if (spin_thread_ && spin_thread_->joinable()) {
    rclcpp::shutdown();
    spin_thread_->join();
  }
}

void ROS2Gimbal::send(bool control, bool fire,
                      float yaw, float /*yaw_vel*/, float /*yaw_acc*/,
                      float pitch, float /*pitch_vel*/, float /*pitch_acc*/)
{
  if (!control) {
    // 一般不需要主动发 0；但如果你们电控要求“持续保持 0”才能停火，可在这里发 0
    return;
  }

  pub_->publish_abs(yaw, pitch);

  // cmd_shoot：你发现它只订阅 UInt8，所以这里用“脉冲式 1”最保险
  if (fire) pub_->publish_shoot(1);
  else      pub_->publish_shoot(0);
}

std::optional<sensor_msgs::msg::Imu> ROS2Gimbal::subscribe_imu()
{
  return sub_->subscribe_imu();
}

std::optional<GimbalJointFeedback> ROS2Gimbal::subscribe_gimbal_joint()
{
  return sub_->subscribe_gimbal_joint();
}

} // namespace io
