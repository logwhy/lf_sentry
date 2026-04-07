#include "ros2.hpp"

#include <rclcpp/rclcpp.hpp>

namespace io
{

ROS2::ROS2()
{
  // Safe to call multiple times in one process; ROS2 will ignore repeats.
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  publish2nav_ = std::make_shared<Publish2Nav>();
  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });
}

ROS2::~ROS2()
{
  // Stop ROS2 and join thread.
  rclcpp::shutdown();
  if (publish_spin_thread_ && publish_spin_thread_->joinable()) {
    publish_spin_thread_->join();
  }
}

void ROS2::publish(const Eigen::Vector4d & target_pos)
{
  if (publish2nav_) {
    publish2nav_->send_data(target_pos);
  }
}

std::vector<int8_t> ROS2::subscribe_enemy_status()
{
  // `sentry_bp` calls this to inform the decider about referee "invincible" state.
  // Your workspace currently has no referee/ROS2 source, so return empty.
  return {};
}

std::vector<int8_t> ROS2::subscribe_autoaim_target()
{
  return {};
}

}  // namespace io