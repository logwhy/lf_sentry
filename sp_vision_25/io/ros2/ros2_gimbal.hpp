#pragma once
#include <memory>
#include <optional>
#include <thread>

#include <sensor_msgs/msg/imu.hpp>

#include "io/ros2/publish2gimbal.hpp"
#include "io/ros2/subscribe2gimbal.hpp"

namespace io {

class ROS2Gimbal {
public:
  ROS2Gimbal();
  ~ROS2Gimbal();

  // 保持你原来自瞄 send 的签名习惯（vel/acc 先不发，避免和下位机控制环打架）
  void send(bool control, bool fire,
            float yaw, float yaw_vel, float yaw_acc,
            float pitch, float pitch_vel, float pitch_acc);

  std::optional<sensor_msgs::msg::Imu> subscribe_imu();
  std::optional<GimbalJointFeedback> subscribe_gimbal_joint();

private:
  std::shared_ptr<Publish2Gimbal> pub_;
  std::shared_ptr<Subscribe2Gimbal> sub_;
  std::unique_ptr<std::thread> spin_thread_;
};

} // namespace io
