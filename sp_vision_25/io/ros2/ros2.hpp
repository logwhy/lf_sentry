#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "publish2nav.hpp"

namespace io
{

// Minimal ROS2 wrapper used by tests/sentry nodes.
// NOTE: The original Tongji version depended on `Subscribe2Nav` / `sp_msgs`,
// which is not available in your workspace. This wrapper keeps only the parts
// that are actually used by `sentry_bp` today.
class ROS2
{
public:
  ROS2();
  ~ROS2();

  // Publish target info for navigation/BT (string topic in Publish2Nav).
  void publish(const Eigen::Vector4d & target_pos);

  // Keep the API used by sentry_bp, but return empty when no referee source exists.
  std::vector<int8_t> subscribe_enemy_status();

  // (kept for compatibility; currently unused / can be wired later)
  std::vector<int8_t> subscribe_autoaim_target();

private:
  std::shared_ptr<Publish2Nav> publish2nav_;
  std::unique_ptr<std::thread> publish_spin_thread_;
};

}  // namespace io

#endif  // IO__ROS2_HPP