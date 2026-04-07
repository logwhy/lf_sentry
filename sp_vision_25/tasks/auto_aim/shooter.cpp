#include "shooter.hpp"

#include <cmath>
#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{

Shooter::Shooter(const std::string & config_path)
: last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);

  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();

  normal_fire_window_ = yaml["normal_fire_window_deg"].as<double>() / 57.3;
  high_speed_fire_window_ = yaml["high_speed_fire_window_deg"].as<double>() / 57.3;
  spin_threshold_ = yaml["spin_threshold_rad"].as<double>();
  tracking_only_fire_ = yaml["tracking_only_fire"].as<bool>();
}

bool Shooter::shoot(
  const io::Command & command,
  const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets,
  const Eigen::Vector3d & gimbal_euler_rad)
{
  return shoot(command, aimer, targets, gimbal_euler_rad, "tracking");
}

bool Shooter::shoot(
  const io::Command & command,
  const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets,
  const Eigen::Vector3d & gimbal_euler_rad,
  const std::string & tracker_state)
{
  if (!auto_fire_ || !command.control || targets.empty()) {
    last_command_ = command;
    return false;
  }

  // 更激进：
  // 原来是 tracker_state != "tracking" 就不打
  // 现在改成只有明确 lost 才不打
  if (tracking_only_fire_ && tracker_state == "lost") {
    last_command_ = command;
    return false;
  }

  if (!aimer.debug_aim_point.valid) {
    last_command_ = command;
    return false;
  }

  const auto & target = targets.front();

  const double target_x = target.ekf_x()[0];
  const double target_y = target.ekf_x()[2];
  const double distance = std::sqrt(tools::square(target_x) + tools::square(target_y));

  // 更激进：整体容差再放大一点
  const double base_tolerance =
    distance > judge_distance_ ? second_tolerance_ : first_tolerance_;
  const double tolerance = base_tolerance * 1.5;

  const double spin_speed = std::abs(target.ekf_x()[7]);
  const double selected_delta_angle = std::abs(aimer.debug_selected_delta_angle);

  // 更激进：高速时仍然区分窗口，但额外再放宽 15%
  const double base_fire_window =
    (spin_speed > spin_threshold_) ? high_speed_fire_window_ : normal_fire_window_;
  const double fire_window = base_fire_window * 1.15;

  // 条件1：装甲板相对正面即可，阈值更宽
  const bool front_enough = selected_delta_angle < fire_window;

  // 条件2：只要求“当前云台角”基本跟上“当前命令角”
  // 不再严苛要求上一帧命令与当前命令必须非常接近
  const bool gimbal_following =
    std::abs(gimbal_euler_rad[0] - command.yaw) < tolerance * 2.0;

  // 保留一个很宽松的防突跳限制，避免纯乱跳
  const bool command_not_too_jump =
    std::abs(last_command_.yaw - command.yaw) < tolerance * 6.0;

  // 近距离额外放行：只要枪口基本跟上，就允许打
  const bool close_range_bonus =
    distance < judge_distance_ * 0.8 &&
    std::abs(gimbal_euler_rad[0] - command.yaw) < tolerance * 2.5;

  const bool can_shoot =
    (front_enough && gimbal_following && command_not_too_jump) ||
    close_range_bonus;

  // tools::logger()->info(
  //   "[Shooter AGG] state={} dist={:.2f} spin={:.2f} delta={:.1f}deg "
  //   "fire_win={:.1f}deg front_ok={} follow_ok={} jump_ok={} close_bonus={} shoot={}",
  //   tracker_state,
  //   distance,
  //   spin_speed,
  //   selected_delta_angle * 57.3,
  //   fire_window * 57.3,
  //   front_enough,
  //   gimbal_following,
  //   command_not_too_jump,
  //   close_range_bonus,
  //   can_shoot);

  last_command_ = command;
  return can_shoot;
}

}  // namespace auto_aim