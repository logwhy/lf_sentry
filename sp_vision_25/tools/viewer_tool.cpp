#include "tools/viewer_tool.hpp"

#include <fmt/core.h>

namespace tools {

ViewerTool::ViewerTool(std::string window_name, double scale)
    : window_name_(std::move(window_name)), scale_(scale), t_disp_fps_(clock_t::now()) {
  // 延迟创建窗口，首次 update 时再 imshow 也可以；这里提前 namedWindow 便于...
  cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
}

void ViewerTool::update_fps() {
  disp_frames_cnt_++;
  const auto now = clock_t::now();
  const double dt = std::chrono::duration<double>(now - t_disp_fps_).count();
  if (dt >= 1.0) {
    disp_fps_ = disp_frames_cnt_ / dt;
    disp_frames_cnt_ = 0;
    t_disp_fps_ = now;
  }
}

void ViewerTool::compose_canvas(const std::vector<cv::Mat>& frames, cv::Mat& out) {
  out.release();
  if (frames.empty()) return;

  if (display_mode_ == 0) {
    // 双画面拼接：优先拼接前两路
    if (frames.size() >= 2) {
      if (!frames[0].empty() && !frames[1].empty()) {
        cv::hconcat(frames[0], frames[1], out);
      } else if (!frames[0].empty()) {
        out = frames[0];
      } else if (!frames[1].empty()) {
        out = frames[1];
      }
    } else {
      // 只有一路
      if (!frames[0].empty()) out = frames[0];
    }
  } else {
    const int idx = display_mode_ - 1;
    if (idx >= 0 && idx < static_cast<int>(frames.size()) && !frames[idx].empty()) {
      out = frames[idx];
    }
  }
}

void ViewerTool::draw_overlay(cv::Mat& img) {
  if (img.empty()) return;
  cv::putText(img, fmt::format("VIEW FPS: {:.1f}", disp_fps_), {10, 30},
              cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
}

bool ViewerTool::update(const std::vector<cv::Mat>& frames) {
  update_fps();

  cv::Mat canvas;
  compose_canvas(frames, canvas);
  if (!canvas.empty()) {
    draw_overlay(canvas);

    cv::Mat resized;
    if (scale_ != 1.0) {
      cv::resize(canvas, resized, {}, scale_, scale_);
      cv::imshow(window_name_, resized);
    } else {
      cv::imshow(window_name_, canvas);
    }
  }

  const char k = static_cast<char>(cv::waitKey(1));
  if (k == 'q') return false;
  if (k == '1') display_mode_ = 1;
  if (k == '2') display_mode_ = 2;
  if (k == 'b') display_mode_ = 0;
  return true;
}

}  // namespace tools