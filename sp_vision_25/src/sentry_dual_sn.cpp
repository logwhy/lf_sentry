//自己写的双摄像头的另一种方案，不是很完善，想复刻perceptron的逻辑，但是不是很成功
//延迟比较高，优点是双摄像头的帧率都能到80帧,但相比较起sentry_dual_cameras识别延迟从6ms升到了20ms
//写的不是很成功说实话。。。。后续如果有人比较了解高性能计算或并行计算也可以搞搞看
#include <fmt/core.h>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <list>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <optional>
#include <yaml-cpp/yaml.h>
#include <future>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"

#include "tasks/omniperception/decider.hpp"     // 决策模块
#include "tasks/omniperception/detection.hpp"   // DetectionResult

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono;

// 小工具：统计窗口（1秒刷新）
struct StatWindow {
  double sum_ms = 0.0;
  double max_ms = 0.0;
  int n = 0;

  void add_ms(double v) {
    sum_ms += v;
    if (v > max_ms) max_ms = v;
    n++;
  }

  void reset() {
    sum_ms = 0.0;
    max_ms = 0.0;
    n = 0;
  }

  double avg_ms() const { return (n > 0) ? (sum_ms / n) : 0.0; }
};

// 相机实例（双相机：cam1=主摄，cam2=后摄辅助）
//  - 每个相机一条常驻推理线程：read -> yolo.detect -> 写入 latest_det（允许丢帧）
//  - 主循环只读取 latest_det（拿到就用，拿不到就 hold） -> Decider::delta_angle -> DetectionResult 队列 -> Decider 过滤/排序/决策
struct CameraInstance {
  std::string name;               // "FRONT" / "BACK"
  std::unique_ptr<io::SNCamera> cam;
  std::unique_ptr<auto_aim::YOLO> yolo;
  double yaw_offset = 0.0;        // 例如后摄 +pi（rad）

  // latest frame（仅用于可视化；use_windows=false 时不会写入）
  mutable std::mutex frame_mtx;
  cv::Mat latest_frame;
  steady_clock::time_point latest_frame_ts{};
  bool has_frame = false;

  // latest detection (producer: infer thread, consumer: main loop)
  struct LatestDet {
    std::list<auto_aim::Armor> armors;
    steady_clock::time_point ts{};
    bool valid = false;
  };

  mutable std::mutex det_mtx;
  LatestDet latest_det;
  steady_clock::time_point last_consumed_ts{};

  // yolo 推理耗时统计（infer thread 写，main thread 读）
  mutable std::mutex yolo_stat_mtx;
  StatWindow yolo_stat;

  // 统计：推理线程自己的计数（用于 read_fps / infer_fps）
  std::atomic<uint64_t> infer_cnt{0};
};

// ===========================
// 可选显示
// ===========================
static void handle_display(const std::vector<cv::Mat>& frames, int& mode, double fps) {
  if (frames.empty()) return;
  cv::Mat canvas;

  if (mode == 0 && frames.size() >= 2) {
    if (!frames[0].empty() && !frames[1].empty()) cv::hconcat(frames[0], frames[1], canvas);
    else if (!frames[0].empty()) canvas = frames[0];
    else if (!frames[1].empty()) canvas = frames[1];
  } else if (mode > 0 && mode <= (int)frames.size()) {
    canvas = frames[mode - 1];
  }

  if (!canvas.empty()) {
    cv::putText(canvas, fmt::format("VIEW FPS: {:.1f}", fps), {10, 30},
                cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
    cv::Mat resized;
    cv::resize(canvas, resized, {}, 0.5, 0.5);
    cv::imshow("Dual Camera View", resized);
  }
}

// 候选结果：每路相机各自产生一个 DetectionResult（和 perceptron 一致），
// 同时保留 cam_idx 便于做“只允许主摄开火”等逻辑。
struct CamDetection {
  size_t cam_idx = 0;
  std::string cam_name;
  omniperception::DetectionResult dr;
};

//（参考 auto_aim_test_all）
static io::Command last_valid_command{};
static int consecutive_lost_frames = 0;
static constexpr int MAX_LOST_FRAMES = 8;
static bool target_stable = false;
static int stable_frame_count = 0;

// 添加角度连续性处理变量
static float last_sent_yaw = 0.0f;
static bool first_yaw_received = false;

// 最后一次“找到有效目标”的时间（用于额外hold）
static steady_clock::time_point last_found_time = steady_clock::now();

static float smooth_yaw_transition(float current_yaw, float last_yaw) {
  float diff = current_yaw - last_yaw;
  while (diff > M_PI) diff -= 2 * M_PI;
  while (diff < -M_PI) diff += 2 * M_PI;
  return last_yaw + diff;
}

int main(int argc, char* argv[]) {
  tools::Exiter exiter;
  std::string config_main = "configs/demo.yaml";

  // --- 读取 use_windows 配置 ---
  bool use_windows = false;
  try {
    use_windows = YAML::LoadFile(config_main)["use_windows"].as<bool>(false);
  } catch (...) {}

  // 云台通信（保持原方式）
  io::Gimbal gimbal(config_main);

  // 用于从 IMU 四元数计算当前云台 yaw（把相对角 delta_yaw 转成绝对 yaw 命令）
  auto_aim::Solver pose_solver(config_main);

  // 决策器：用来在两路相机候选之间选择“转动角度来自哪个相机”
  omniperception::Decider decider(config_main);

  // yaw_offset 维持你原来：后摄 +pi
  std::vector<std::unique_ptr<CameraInstance>> cams;
  cams.reserve(2);
  std::vector<std::pair<std::string, double>> cfg_list = {
    {"configs/cam1.yaml", 0.0},
    {"configs/cam2.yaml", M_PI}
  };

  for (size_t i = 0; i < cfg_list.size(); ++i) {
    auto ci = std::make_unique<CameraInstance>();
    ci->name = (i == 0 ? "FRONT" : "BACK");
    ci->cam = std::make_unique<io::SNCamera>(cfg_list[i].first);
    ci->yolo = std::make_unique<auto_aim::YOLO>(cfg_list[i].first);
    ci->yaw_offset = cfg_list[i].second;

    //tools::logger()->info("初始化相机: {}", ci->name);
    cams.emplace_back(std::move(ci));

    std::this_thread::sleep_for(seconds(2));
  }

  const size_t MAIN_CAM_IDX = 0; // cam1 = FRONT（主摄，可开火）
  const size_t AUX_CAM_IDX  = 1; // cam2 = BACK（后摄，仅辅助识别/转动）

  // 相机常驻推理线程：每个相机自己 read -> yolo.detect -> 写 latest_det（允许丢帧）
  std::vector<std::thread> threads;
  threads.reserve(cams.size());
  for (auto& ci_ptr : cams) {
    threads.emplace_back([&ci_ptr, &exiter, use_windows]() {
      cv::Mat img;
      steady_clock::time_point ts;
      while (!exiter.exit()) {
        ci_ptr->cam->read(img, ts);
        if (img.empty()) {
          std::this_thread::sleep_for(milliseconds(1));
          continue;
        }

        // 仅在需要显示时缓存一份图像（避免无谓 clone）
        // 注意：use_windows 通常为 false（实战关 GUI 提升帧率）
        if (use_windows) {
          std::lock_guard<std::mutex> lk(ci_ptr->frame_mtx);
          ci_ptr->latest_frame = img.clone();
          ci_ptr->latest_frame_ts = ts;
          ci_ptr->has_frame = true;
        }

        // YOLO detect（在推理线程里做）
        std::list<auto_aim::Armor> armors;
        auto t0 = steady_clock::now();
        armors = ci_ptr->yolo->detect(img, 0);
        auto t1 = steady_clock::now();
        {
          std::lock_guard<std::mutex> lk(ci_ptr->yolo_stat_mtx);
          ci_ptr->yolo_stat.add_ms(duration<double, std::milli>(t1 - t0).count());
        }

        // 写入“最新结果槽”（主线程只取最新，不等待）
        {
          std::lock_guard<std::mutex> lk(ci_ptr->det_mtx);
          ci_ptr->latest_det.armors = std::move(armors);
          ci_ptr->latest_det.ts = ts;
          ci_ptr->latest_det.valid = true;
        }

        ci_ptr->infer_cnt.fetch_add(1, std::memory_order_relaxed);
      }
    });
  }

  // Debug统计变量
  using clock_t = std::chrono::steady_clock;
  auto t_report = clock_t::now();

  int main_loop_cnt = 0;
  double main_fps = 0.0;

  std::vector<int> cam_frames_cnt(cams.size(), 0);
  std::vector<double> cam_last_age_ms(cams.size(), -1.0);

  StatWindow stat_q_ms;
  StatWindow stat_send_ms;

  int found_cnt = 0;
  int lost_cnt = 0;

  // 显示相关
  int display_mode = 0;
  int disp_frames_cnt = 0;
  auto t_disp_fps = clock_t::now();
  double disp_fps = 0.0;

  while (!exiter.exit()) {
    main_loop_cnt++;

    std::vector<cv::Mat> current_frames;
    if (use_windows) current_frames.resize(cams.size());

    Eigen::Quaterniond q_now;
    {
      auto t0 = clock_t::now();
      q_now = gimbal.q(clock_t::now());
      auto t1 = clock_t::now();
      stat_q_ms.add_ms(duration<double, std::milli>(t1 - t0).count());
    }

    // 当前云台 yaw（rad）
    pose_solver.set_R_gimbal2world(q_now);
    const Eigen::Vector3d gimbal_pos = tools::eulers(pose_solver.R_gimbal2world(), 2, 1, 0);

    // ============
    // perceptron 同款：YOLO -> delta_angle -> DetectionResult
    // ============
    std::vector<CamDetection> dets;
    dets.reserve(cams.size());

    for (size_t i = 0; i < cams.size(); ++i) {
      static int back_tick = 0;
      if (i == AUX_CAM_IDX) {
        back_tick++;
        if (back_tick % 3 != 0) continue; // BACK 只跑 1/3 帧（减负）
      }

      // 从推理线程的“最新结果槽”取数据（不等待；允许丢帧）
      CameraInstance::LatestDet det;
      bool has_new = false;
      {
        std::lock_guard<std::mutex> lk(cams[i]->det_mtx);
        if (cams[i]->latest_det.valid && cams[i]->latest_det.ts != cams[i]->last_consumed_ts) {
          det = cams[i]->latest_det;  // 拷贝一份（主线程不阻塞推理线程）
          cams[i]->last_consumed_ts = cams[i]->latest_det.ts;
          has_new = true;
        }
      }

      if (!has_new) {
        continue;  // 本轮没有新结果：主线程会沿用“hold 上一次命令”的逻辑
      }

      const auto ts = det.ts;
      auto armors = det.armors;

      cam_frames_cnt[i] += 1;
      auto now = clock_t::now();
      cam_last_age_ms[i] = duration<double, std::milli>(now - ts).count();

      if (use_windows) {
        std::lock_guard<std::mutex> lk(cams[i]->frame_mtx);
        if (cams[i]->has_frame) current_frames[i] = cams[i]->latest_frame;
      }

      if (armors.empty()) continue;

      // camera tag：使用 "front"/"back"，对应你需要在 decider.delta_angle 里补分支
      const std::string cam_tag = (i == MAIN_CAM_IDX ? "front" : "back");
      const Eigen::Vector2d da_deg = decider.delta_angle(armors, cam_tag);

      omniperception::DetectionResult dr;
      dr.armors = armors;
      dr.timestamp = ts;
      dr.delta_yaw = tools::limit_rad(da_deg[0] / 57.3);
      dr.delta_pitch = tools::limit_rad(da_deg[1] / 57.3);

      dets.push_back(CamDetection{i, cams[i]->name, dr});
    }

    // ============
    // 用 decider 的函数过滤/优先级 + 排序（保留 cam_idx 映射）
    // ============
    for (auto & cd : dets) {
      decider.armor_filter(cd.dr.armors);
      decider.set_priority(cd.dr.armors);
      if (!cd.dr.armors.empty()) {
        cd.dr.armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) {
          return a.priority < b.priority;
        });
      }
    }
    dets.erase(std::remove_if(dets.begin(), dets.end(), [](const CamDetection & cd) {
      return cd.dr.armors.empty();
    }), dets.end());

    std::sort(dets.begin(), dets.end(), [](const CamDetection & a, const CamDetection & b) {
      return a.dr.armors.front().priority < b.dr.armors.front().priority;
    });

    std::vector<omniperception::DetectionResult> detection_queue;
    detection_queue.reserve(dets.size());
    for (const auto & cd : dets) detection_queue.push_back(cd.dr);

    bool found = !detection_queue.empty();
    io::Command out_cmd = decider.decide(detection_queue); // yaw/pitch = 相对角(rad)
    size_t chosen_cam_idx = found ? dets.front().cam_idx : MAIN_CAM_IDX;

    // ============
    // 防丢帧 + 稳定开火（保留原风格）
    // ============
    if (found) {
      last_found_time = steady_clock::now();
      found_cnt++;

      const bool has_valid_target = out_cmd.control;
      if (has_valid_target) {
        consecutive_lost_frames = 0;

        // 不再区分相机来源：只要本帧有有效目标就累计稳定性
        stable_frame_count++;
        if (stable_frame_count >= 3) target_stable = true;

        if (std::isfinite(out_cmd.yaw) && std::isfinite(out_cmd.pitch)) {
          last_valid_command = out_cmd;
        }
      } else {
        consecutive_lost_frames++;
        stable_frame_count = 0;
        target_stable = false;
      }

      out_cmd.control = true;
      if (!std::isfinite(out_cmd.yaw) || !std::isfinite(out_cmd.pitch)) {
        out_cmd = last_valid_command;
        out_cmd.control = true;
        out_cmd.shoot = false;
        tools::logger()->warn("Invalid command, using last valid command");
      }

      // 开火只允许主摄 + 稳定 + yaw 波动小
      // （如果你想强制“只有 chosen_cam_idx==MAIN_CAM_IDX 才能 shoot”，可再加条件）
      if (target_stable &&
          std::abs(out_cmd.yaw - last_valid_command.yaw) * 57.3 < 5.0) {
        out_cmd.shoot = true;
      } else {
        out_cmd.shoot = false;
      }

    } else {
      lost_cnt++;

      consecutive_lost_frames++;
      stable_frame_count = 0;
      target_stable = false;

      if (consecutive_lost_frames <= MAX_LOST_FRAMES) {
        out_cmd = last_valid_command;
        out_cmd.control = true;
        out_cmd.shoot = false;
      } else {
        out_cmd.control = false;
        out_cmd.shoot = false;
        out_cmd.yaw = 0;
        out_cmd.pitch = 0;
      }

      auto now = steady_clock::now();
      auto since_found_ms = duration_cast<milliseconds>(now - last_found_time).count();
      const int HOLD_MS = 200;
      if (since_found_ms > HOLD_MS) {
        out_cmd.control = false;
        out_cmd.shoot = false;
        out_cmd.yaw = 0;
        out_cmd.pitch = 0;
      }
    }

    // ============
    // send（保持原通信方式）
    // ============
      // 相对 yaw -> 绝对 yaw（对齐 decider.cpp 的旧做法：yaw 加 gimbal_pos[0]，pitch 不加）
    if (out_cmd.control) {
      out_cmd.yaw = tools::limit_rad(out_cmd.yaw + gimbal_pos[0]);
      // yaw 连续性处理
      if (first_yaw_received) {
        out_cmd.yaw = smooth_yaw_transition(out_cmd.yaw, last_sent_yaw);
      } else {
        first_yaw_received = true;
      }
      last_sent_yaw = out_cmd.yaw;

      auto t0 = clock_t::now();
      gimbal.send(out_cmd.control, out_cmd.shoot, out_cmd.yaw, 0, 0, out_cmd.pitch, 0, 0);
      auto t1 = clock_t::now();
      stat_send_ms.add_ms(duration<double, std::milli>(t1 - t0).count());
    }

    // ============
    // 显示帧率（可选）
    // ============
    if (use_windows) {
      disp_frames_cnt++;
      auto now = clock_t::now();
      auto dt = duration<double>(now - t_disp_fps).count();
      if (dt >= 1.0) {
        disp_fps = disp_frames_cnt / dt;
        disp_frames_cnt = 0;
        t_disp_fps = now;
      }

      handle_display(current_frames, display_mode, disp_fps);
      char k = (char)cv::waitKey(1);
      if (k == 'q') break;
      if (k == '1') display_mode = 1;
      if (k == '2') display_mode = 2;
      if (k == 'b') display_mode = 0;
    }

    // ============
    // 1Hz 报告
    // ============
    auto now = clock_t::now();
    auto dt_rep = duration<double>(now - t_report).count();
    if (dt_rep >= 1.0) {
      main_fps = main_loop_cnt / dt_rep;

      std::cout << "\n========== DEBUG REPORT (1s) ==========\n";
      std::cout << fmt::format("Main loop FPS: {:.2f}\n", main_fps);

      for (size_t i = 0; i < cams.size(); ++i) {
        double cam_fps = cam_frames_cnt[i] / dt_rep;
        std::cout << fmt::format(
          "Cam[{}:{}] read_fps={:.2f}, last_frame_age={:.1f} ms\n",
          i, cams[i]->name, cam_fps, cam_last_age_ms[i]
        );
      }

      std::cout << fmt::format(
        "gimbal.q()      avg={:.3f} ms, max={:.3f} ms\n",
        stat_q_ms.avg_ms(), stat_q_ms.max_ms
      );

      // 聚合各相机推理线程的 YOLO 耗时统计
      double yolo_sum_ms = 0.0;
      double yolo_max_ms = 0.0;
      int yolo_n = 0;
      for (auto & ci : cams) {
        std::lock_guard<std::mutex> lk(ci->yolo_stat_mtx);
        yolo_sum_ms += ci->yolo_stat.sum_ms;
        yolo_max_ms = std::max(yolo_max_ms, ci->yolo_stat.max_ms);
        yolo_n += ci->yolo_stat.n;
      }
      const double yolo_avg_ms = (yolo_n > 0) ? (yolo_sum_ms / yolo_n) : 0.0;

      std::cout << fmt::format(
        "yolo.detect()   avg={:.3f} ms, max={:.3f} ms\n",
        yolo_avg_ms, yolo_max_ms
      );

      std::cout << fmt::format(
        "gimbal.send()   avg={:.3f} ms, max={:.3f} ms\n",
        stat_send_ms.avg_ms(), stat_send_ms.max_ms
      );

      std::cout << fmt::format(
        "found_cnt={}, lost_cnt={}, stable={}, lost_frames={}/{}\n",
        found_cnt, lost_cnt, target_stable, consecutive_lost_frames, MAX_LOST_FRAMES
      );
      std::cout << "=======================================\n";

      // reset
      t_report = now;
      main_loop_cnt = 0;
      for (auto& c : cam_frames_cnt) c = 0;

      stat_q_ms.reset();
      for (auto & ci : cams) {
        std::lock_guard<std::mutex> lk(ci->yolo_stat_mtx);
        ci->yolo_stat.reset();
      }
      stat_send_ms.reset();

      found_cnt = 0;
      lost_cnt = 0;
    }
  }

  for (auto& t : threads) t.join();
  return 0;
}
