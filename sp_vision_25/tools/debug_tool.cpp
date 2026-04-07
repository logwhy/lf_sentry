#include "tools/debug_tool.hpp"

#include <fmt/core.h>
#include <iostream>

namespace tools {

void StatWindow::add_ms(double v) {
  sum_ms_ += v;
  if (v > max_ms_) max_ms_ = v;
  n_++;
}

void StatWindow::reset() {
  sum_ms_ = 0.0;
  max_ms_ = 0.0;
  n_ = 0;
}

double StatWindow::avg_ms() const {
  return (n_ > 0) ? (sum_ms_ / n_) : 0.0;
}

DebugTool::DebugTool(std::size_t cam_count)
    : cam_names_(cam_count),
      cam_frames_cnt_(cam_count, 0),
      cam_last_age_ms_(cam_count, -1.0),
      t_report_(clock_t::now()) {}

void DebugTool::set_cam_name(std::size_t idx, std::string name) {
  if (idx >= cam_names_.size()) return;
  cam_names_[idx] = std::move(name);
}

void DebugTool::tick_main_loop() { main_loop_cnt_++; }

void DebugTool::record_cam_frame(std::size_t idx, double last_frame_age_ms) {
  if (idx >= cam_frames_cnt_.size()) return;
  cam_frames_cnt_[idx] += 1;
  cam_last_age_ms_[idx] = last_frame_age_ms;
}

void DebugTool::add_gimbal_q_ms(double ms) { stat_q_ms_.add_ms(ms); }
void DebugTool::add_yolo_detect_ms(double ms) { stat_yolo_ms_.add_ms(ms); }
void DebugTool::add_gimbal_send_ms(double ms) { stat_send_ms_.add_ms(ms); }

void DebugTool::add_found() { found_cnt_++; }
void DebugTool::add_lost() { lost_cnt_++; }

void DebugTool::report_if_due(bool target_stable,
                              int consecutive_lost_frames,
                              int max_lost_frames) {
  const auto now = clock_t::now();
  const double dt_rep = std::chrono::duration<double>(now - t_report_).count();
  if (dt_rep < 1.0) return;

  const double main_fps = (dt_rep > 0) ? (main_loop_cnt_ / dt_rep) : 0.0;

  std::cout << "\n========== DEBUG REPORT (1s) ==========\n";
  std::cout << fmt::format("Main loop FPS: {:.2f}\n", main_fps);

  for (std::size_t i = 0; i < cam_frames_cnt_.size(); ++i) {
    const double cam_fps = cam_frames_cnt_[i] / dt_rep;
    const std::string name = cam_names_[i].empty() ? fmt::format("Cam{}", i) : cam_names_[i];
    std::cout << fmt::format(
        "Cam[{}:{}] read_fps={:.2f}, last_frame_age={:.1f} ms\n",
        i, name, cam_fps, cam_last_age_ms_[i]);
  }

  std::cout << fmt::format("gimbal.q()      avg={:.3f} ms, max={:.3f} ms\n",
                           stat_q_ms_.avg_ms(), stat_q_ms_.max_ms());
  std::cout << fmt::format("yolo.detect()   avg={:.3f} ms, max={:.3f} ms\n",
                           stat_yolo_ms_.avg_ms(), stat_yolo_ms_.max_ms());
  std::cout << fmt::format("gimbal.send()   avg={:.3f} ms, max={:.3f} ms\n",
                           stat_send_ms_.avg_ms(), stat_send_ms_.max_ms());

  std::cout << fmt::format(
      "found_cnt={}, lost_cnt={}, stable={}, lost_frames={}/{}\n",
      found_cnt_, lost_cnt_, target_stable, consecutive_lost_frames,
      max_lost_frames);
  std::cout << "=======================================\n";

  t_report_ = now;
  reset_windows();
}

void DebugTool::reset_windows() {
  main_loop_cnt_ = 0;
  for (auto &c : cam_frames_cnt_) c = 0;

  stat_q_ms_.reset();
  stat_yolo_ms_.reset();
  stat_send_ms_.reset();

  found_cnt_ = 0;
  lost_cnt_ = 0;
}

}  // namespace tools