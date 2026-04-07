#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

namespace tools {

class StatWindow {
public:
  void add_ms(double v);
  void reset();
  double avg_ms() const;
  double max_ms() const { return max_ms_; }

private:
  double sum_ms_{0.0};
  double max_ms_{0.0};
  int n_{0};
};

class DebugTool {
public:
  using clock_t = std::chrono::steady_clock;

  explicit DebugTool(std::size_t cam_count);

  void set_cam_name(std::size_t idx, std::string name);

  void tick_main_loop();
  void record_cam_frame(std::size_t idx, double last_frame_age_ms);

  void add_gimbal_q_ms(double ms);
  void add_yolo_detect_ms(double ms);
  void add_gimbal_send_ms(double ms);

  void add_found();
  void add_lost();

  void report_if_due(bool target_stable,
                     int consecutive_lost_frames,
                     int max_lost_frames);

private:
  void reset_windows();

  std::vector<std::string> cam_names_;
  std::vector<int> cam_frames_cnt_;
  std::vector<double> cam_last_age_ms_;

  StatWindow stat_q_ms_;
  StatWindow stat_yolo_ms_;
  StatWindow stat_send_ms_;

  clock_t::time_point t_report_;
  int main_loop_cnt_{0};

  int found_cnt_{0};
  int lost_cnt_{0};
};

}  // namespace tools