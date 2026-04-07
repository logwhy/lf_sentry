#pragma once

#include <Eigen/Core>
#include <list>
#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/target.hpp"

namespace auto_aim
{

class Shooter
{
public:
  explicit Shooter(const std::string & config_path);

  // 新接口
  bool shoot(
    const io::Command & command,
    const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets,
    const Eigen::Vector3d & gimbal_euler_rad,
    const std::string & tracker_state);

  // 兼容旧代码的接口
  bool shoot(
    const io::Command & command,
    const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets,
    const Eigen::Vector3d & gimbal_euler_rad);

private:
  double first_tolerance_{0.0};
  double second_tolerance_{0.0};
  double judge_distance_{2.0};
  bool auto_fire_{true};

  double normal_fire_window_{0.0};
  double high_speed_fire_window_{0.0};
  double spin_threshold_{1.2};
  bool tracking_only_fire_{true};

  io::Command last_command_{false, false, 0, 0};
};

}  // namespace auto_aim