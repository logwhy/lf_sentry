#pragma once
#include <chrono>
#include "io/gimbal/gimbal.hpp"

class AimGuard {
public:
  using clock_t = std::chrono::steady_clock;

  struct Params {
    int max_lost_frames = 8;
    int hold_ms = 200;
    int stable_need_frames = 3;
    double stable_yaw_deg = 5.0;
    bool enable_stable_fire = true;
  };

  explicit AimGuard(Params p = Params{});

  io::Command process(bool found, io::Command cmd);

  bool target_stable() const { return target_stable_; }
  int lost_frames() const { return consecutive_lost_frames_; }

private:
  Params p_;
  io::Command last_valid_command_{};
  int consecutive_lost_frames_ = 0;
  bool target_stable_ = false;
  int stable_frame_count_ = 0;
  clock_t::time_point last_found_time_;
};