#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace tools {

class ViewerTool {
public:
  using clock_t = std::chrono::steady_clock;

  ViewerTool(std::string window_name = "Dual Camera View", double scale = 0.5);

  // 更新显示；返回 false 表示用户请求退出（按 q）
  bool update(const std::vector<cv::Mat>& frames);

  // 0=拼接(双画面)；1..N=单画面
  void set_mode(int mode) { display_mode_ = mode; }
  int mode() const { return display_mode_; }

  // 当前显示帧率（1Hz 更新）
  double fps() const { return disp_fps_; }

private:
  void compose_canvas(const std::vector<cv::Mat>& frames, cv::Mat& out);
  void draw_overlay(cv::Mat& img);
  void update_fps();

  std::string window_name_;
  double scale_{0.5};

  int display_mode_{0};

  int disp_frames_cnt_{0};
  clock_t::time_point t_disp_fps_;
  double disp_fps_{0.0};
};

}  // namespace tools