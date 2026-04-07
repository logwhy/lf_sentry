#include "aim_guard.hpp"
#include <cmath>

AimGuard::AimGuard(Params p) : p_(p), last_found_time_(clock_t::now()) {}

io::Command AimGuard::process(bool found, io::Command cmd) {
  if (found) {
    last_found_time_ = clock_t::now();
    if (cmd.control) {
      consecutive_lost_frames_ = 0;
      stable_frame_count_++;
      if (stable_frame_count_ >= p_.stable_need_frames) target_stable_ = true;
      if (std::isfinite(cmd.yaw) && std::isfinite(cmd.pitch)) last_valid_command_ = cmd;
    } else {
      consecutive_lost_frames_++;
      stable_frame_count_ = 0;
      target_stable_ = false;
    }
    cmd.control = true;
    if (!std::isfinite(cmd.yaw) || !std::isfinite(cmd.pitch)) {
      cmd = last_valid_command_;
      cmd.control = true;
      cmd.shoot = false;
    }
    if (p_.enable_stable_fire && target_stable_) {
      double yaw_diff_deg = std::abs(cmd.yaw - last_valid_command_.yaw) * 57.3;
      cmd.shoot = (yaw_diff_deg < p_.stable_yaw_deg);
    } else {
      cmd.shoot = false;
    }
    return cmd;
  }

  consecutive_lost_frames_++;
  stable_frame_count_ = 0;
  target_stable_ = false;
  if (consecutive_lost_frames_ <= p_.max_lost_frames) {
    cmd = last_valid_command_;
    cmd.control = true;
    cmd.shoot = false;
  } else {
    cmd.control = false; cmd.shoot = false; cmd.yaw = 0; cmd.pitch = 0;
  }

  auto since_found_ms = std::chrono::duration_cast<std::chrono::milliseconds>(clock_t::now() - last_found_time_).count();
  if (since_found_ms > p_.hold_ms) {
    cmd.control = false; cmd.shoot = false; cmd.yaw = 0; cmd.pitch = 0;
  }
  return cmd;
}