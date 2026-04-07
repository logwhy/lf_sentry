#include <fmt/core.h>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <list>
#include <atomic>
#include <cmath>
#include <mutex>
#include <optional>
#include <future>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/detection.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/debug_tool.hpp"
#include "tools/viewer_tool.hpp"

using namespace std::chrono;

// 相机实例：包含相机驱动、YOLO网络及最新帧缓存
struct CameraInstance {
  std::string name;               // "FRONT" / "BACK"
  std::unique_ptr<io::SNCamera> cam;
  std::unique_ptr<auto_aim::YOLO> yolo;
  double yaw_offset = 0.0;

  std::mutex latest_mtx;
  cv::Mat latest_frame;
  steady_clock::time_point latest_ts{}, last_processed_ts{};
  bool has_latest = false;
};

// 单路检测结果
struct CamDetection {
  size_t cam_idx = 0;
  std::string cam_name;
  omniperception::DetectionResult dr;
};

// 全局状态变量
static io::Command last_valid_command{};
static int consecutive_lost_frames = 0;
static constexpr int MAX_LOST_FRAMES = 8;
static bool target_stable = false;
static int stable_frame_count = 0;
static float last_sent_yaw = 0.0f;
static bool first_yaw_received = false;
static steady_clock::time_point last_found_time = steady_clock::now();

// 角度平滑过渡
static float smooth_yaw_transition(float current, float last) {
  float diff = current - last;
  while (diff > M_PI) diff -= 2 * M_PI;
  while (diff < -M_PI) diff += 2 * M_PI;
  return last + diff;
}

int main(int argc, char* argv[]) {
  tools::Exiter exiter;
  std::string config_main = "configs/demo.yaml";
  
  bool use_windows = false;
  try { use_windows = YAML::LoadFile(config_main)["use_windows"].as<bool>(false); } catch (...) {}

  // 初始化核心模块
  io::Gimbal gimbal(config_main);
  auto_aim::Solver pose_solver(config_main);
  omniperception::Decider decider(config_main);

  // 初始化双相机 (0:FRONT, 1:BACK)
  std::vector<std::unique_ptr<CameraInstance>> cams;
  std::vector<std::pair<std::string, double>> cfg_list = {{"configs/cam1.yaml", 0.0}, {"configs/cam2.yaml", M_PI}};
  for (size_t i = 0; i < cfg_list.size(); ++i) {
    auto ci = std::make_unique<CameraInstance>();
    ci->name = (i == 0 ? "FRONT" : "BACK");
    ci->cam = std::make_unique<io::SNCamera>(cfg_list[i].first);
    ci->yolo = std::make_unique<auto_aim::YOLO>(cfg_list[i].first);
    ci->yaw_offset = cfg_list[i].second;
    cams.emplace_back(std::move(ci));
    std::this_thread::sleep_for(seconds(2));
  }

  const size_t MAIN_CAM_IDX = 0; 
  const size_t AUX_CAM_IDX  = 1;

  // 启动读帧线程
  std::vector<std::thread> threads;
  for (auto& ci : cams) {
    threads.emplace_back([&ci, &exiter]() {
      cv::Mat f; steady_clock::time_point t;
      while (!exiter.exit()) {
        ci->cam->read(f, t);
        if (!f.empty()) {
          std::lock_guard<std::mutex> lk(ci->latest_mtx);
          ci->latest_frame = f.clone(); ci->latest_ts = t; ci->has_latest = true;
        } else {
          std::this_thread::sleep_for(milliseconds(1));
        }
      }
    });
  }

  // Debug & View 工具
  using clock_t = std::chrono::steady_clock;
  tools::DebugTool debug(cams.size());
  for (size_t i = 0; i < cams.size(); ++i) debug.set_cam_name(i, cams[i]->name);

  std::unique_ptr<tools::ViewerTool> viewer;
  std::vector<cv::Mat> current_frames(cams.size());
  if (use_windows) viewer = std::make_unique<tools::ViewerTool>("BP Test View", 0.5);

  // --- 主循环 ---
  while (!exiter.exit()) {
    debug.tick_main_loop();
    if (use_windows && current_frames.size() != cams.size()) current_frames.resize(cams.size());

    // 1. 获取云台姿态
    auto t0 = clock_t::now();
    Eigen::Quaterniond q_now = gimbal.q(clock_t::now());
    debug.add_gimbal_q_ms(duration<double, std::milli>(clock_t::now() - t0).count());
    
    pose_solver.set_R_gimbal2world(q_now);
    const Eigen::Vector3d gimbal_pos = tools::eulers(pose_solver.R_gimbal2world(), 2, 1, 0);

    // 2. 定义检测流程
    std::vector<CamDetection> dets;
    dets.reserve(2);

    auto run_detect = [&](size_t i) {
      cv::Mat img; steady_clock::time_point ts;
      {
        std::lock_guard<std::mutex> lk(cams[i]->latest_mtx);
        if (!cams[i]->has_latest || cams[i]->latest_ts == cams[i]->last_processed_ts) return false;
        img = cams[i]->latest_frame; ts = cams[i]->latest_ts;
        cams[i]->last_processed_ts = ts;
      }
      if (img.empty()) return false;
      
      debug.record_cam_frame(i, duration<double, std::milli>(clock_t::now() - ts).count());
      if (use_windows) current_frames[i] = img;

      auto t_yolo = clock_t::now();
      auto armors = cams[i]->yolo->detect(img, 0);
      debug.add_yolo_detect_ms(duration<double, std::milli>(clock_t::now() - t_yolo).count());

      if (armors.empty()) return false;

      const std::string tag = (i == MAIN_CAM_IDX ? "front" : "back");
      Eigen::Vector2d da = decider.delta_angle(armors, tag);
      
      omniperception::DetectionResult dr;
      dr.armors = armors; dr.timestamp = ts;
      dr.delta_yaw = tools::limit_rad(da[0] / 57.3);
      dr.delta_pitch = tools::limit_rad(da[1] / 57.3);
      dets.push_back({i, cams[i]->name, dr});
      return true;
    };

    auto filter_and_sort = [&]() {
      for (auto& cd : dets) {
        decider.armor_filter(cd.dr.armors);
        decider.set_priority(cd.dr.armors);
        cd.dr.armors.sort([](const auto& a, const auto& b){ return a.priority < b.priority; });
      }
      dets.erase(std::remove_if(dets.begin(), dets.end(), [](const auto& cd){ return cd.dr.armors.empty(); }), dets.end());
    };

    // 3. 执行策略：主摄 -> 过滤 -> (若无) -> 后摄 -> 过滤
    run_detect(MAIN_CAM_IDX);
    filter_and_sort();
    
    if (dets.empty()) {
      run_detect(AUX_CAM_IDX);
      filter_and_sort();
    }

    if (!dets.empty()) {
      std::sort(dets.begin(), dets.end(), [](const auto& a, const auto& b) {
        return a.dr.armors.front().priority < b.dr.armors.front().priority;
      });
    }

    // 4. 最终决策
    std::vector<omniperception::DetectionResult> q;
    for (const auto& cd : dets) q.push_back(cd.dr);
    
    bool found = !q.empty();
    io::Command cmd = decider.decide(q);

    // 5. 状态机与控制逻辑
    if (found) {
      last_found_time = steady_clock::now();
      debug.add_found();

      if (cmd.control) {
        consecutive_lost_frames = 0;
        if (++stable_frame_count >= 3) target_stable = true;
        if (std::isfinite(cmd.yaw) && std::isfinite(cmd.pitch)) last_valid_command = cmd;
      } else {
        consecutive_lost_frames++;
        stable_frame_count = 0; 
        target_stable = false;
      }

      cmd.control = true;
      if (!std::isfinite(cmd.yaw) || !std::isfinite(cmd.pitch)) {
        cmd = last_valid_command;
        cmd.control = true; cmd.shoot = false;
        tools::logger()->warn("Invalid command, using last valid");
      }

      // 开火条件：稳定且抖动小
      bool aim_stable = std::abs(cmd.yaw - last_valid_command.yaw) * 57.3 < 3.0 && std::abs(cmd.yaw) * 57.3 < 3.0;
      cmd.shoot = target_stable && aim_stable;

    } else {
      debug.add_lost();
      consecutive_lost_frames++;
      stable_frame_count = 0; 
      target_stable = false;

      if (consecutive_lost_frames <= MAX_LOST_FRAMES) {
        cmd = last_valid_command;
        cmd.control = true; cmd.shoot = false;
      } else {
        cmd = {false, false, 0, 0}; // Reset
      }

      // 额外保持时间
      if (duration_cast<milliseconds>(steady_clock::now() - last_found_time).count() > 200) {
        cmd = {false, false, 0, 0};
      }
    }

    // 6. 发送指令
    if (cmd.control) {
      cmd.yaw = tools::limit_rad(cmd.yaw + gimbal_pos[0]); // 相对转绝对
      
      if (first_yaw_received) cmd.yaw = smooth_yaw_transition(cmd.yaw, last_sent_yaw);
      else first_yaw_received = true;
      last_sent_yaw = cmd.yaw;

      auto t_send = clock_t::now();
      gimbal.send(cmd.control, cmd.shoot, cmd.yaw, 0, 0, cmd.pitch, 0, 0);
      debug.add_gimbal_send_ms(duration<double, std::milli>(clock_t::now() - t_send).count());
    }

    // 7. 更新显示与报告
    if (use_windows) {
      if (!viewer->update(current_frames)) break;
    }
    debug.report_if_due(target_stable, consecutive_lost_frames, MAX_LOST_FRAMES);

  } // End while (fixed)

  for (auto& t : threads) t.join();
  return 0;
}