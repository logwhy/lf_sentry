#include <fmt/core.h>

#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <memory>

// 引入你的工程头文件
#include "io/camera.hpp"
#include "io/ros2/gimbal_ros.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/debug_tool.hpp"

using namespace std::chrono;
using SteadyClock = std::chrono::steady_clock;

// 简单的耗时统计辅助类
struct ScopedTimer {
    double& out_ms;
    SteadyClock::time_point start;
    ScopedTimer(double& target) : out_ms(target), start(SteadyClock::now()) {}
    ~ScopedTimer() {
        out_ms = std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
    }
};

class AsyncCamera {
public:
    // idx: 0 for front, 1 for back
    AsyncCamera(const std::string& config_path, tools::DebugTool* debug_tool, int cam_idx)
        : cam_(config_path), debug_(debug_tool), idx_(cam_idx) {

        debug_->set_cam_name(idx_, (idx_ == 0 ? "Front" : "Back"));
        running_ = true;
        thread_ = std::thread(&AsyncCamera::update, this);
    }

    ~AsyncCamera() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    // 获取最新图像 (非阻塞，极快)
    bool get(cv::Mat& out_img, SteadyClock::time_point& out_stamp) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_img_.empty()) return false;

        latest_img_.copyTo(out_img);
        out_stamp = latest_stamp_;
        return true;
    }

private:
    void update() {
        while (running_) {
            cv::Mat temp_img;
            SteadyClock::time_point temp_stamp;

            cam_.read(temp_img, temp_stamp);

            if (!temp_img.empty()) {
                double latency = std::chrono::duration<double, std::milli>(
                    SteadyClock::now() - temp_stamp).count();

                if (debug_) debug_->record_cam_frame(idx_, latency);

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_img_ = temp_img;
                    latest_stamp_ = temp_stamp;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    io::Camera cam_;
    tools::DebugTool* debug_;
    int idx_;

    std::thread thread_;
    std::mutex mutex_;
    std::atomic<bool> running_;

    cv::Mat latest_img_;
    SteadyClock::time_point latest_stamp_;
};

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  tools::Exiter exiter;
  tools::Recorder recorder;

  // 1. 初始化 DebugTool (2个相机)
  tools::DebugTool debug_tool(2);

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) { cli.printMessage(); return 0; }
  auto config_path = cli.get<std::string>(0);

  auto gimbal = std::make_shared<io::GimbalROS>();
  gimbal->start_spin();

  // 2. 初始化异步相机
  fmt::print("Initializing Async Cameras...\n");

  // 主摄必须成功
  AsyncCamera front_cam("configs/cam1.yaml", &debug_tool, 0);

  // 后摄可选
  std::unique_ptr<AsyncCamera> back_cam;
  bool back_cam_available = false;

  try {
    back_cam = std::make_unique<AsyncCamera>("configs/cam2.yaml", &debug_tool, 1);
    back_cam_available = true;
    fmt::print("[INFO] Back camera initialized successfully.\n");
  } catch (const std::exception &e) {
    back_cam_available = false;
    fmt::print("[WARN] Back camera init failed: {}\n", e.what());
    fmt::print("[WARN] System will continue with front camera only.\n");
  } catch (...) {
    back_cam_available = false;
    fmt::print("[WARN] Back camera init failed with unknown error.\n");
    fmt::print("[WARN] System will continue with front camera only.\n");
  }

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  omniperception::Decider decider(config_path);

  cv::Mat img_front, img_back;
  SteadyClock::time_point ts_front, ts_back;
  io::Command command{false, false, 0, 0};

  double t_q = 0, t_yolo = 0, t_send = 0;

  // ===== 新增：用于目标坐标广播 =====
  bool need_publish_target_xyz = false;
  Eigen::Vector3d publish_xyz_camera = Eigen::Vector3d::Zero();
  Eigen::Vector3d publish_xyz_gimbal = Eigen::Vector3d::Zero();

  // ===== 新增：用于丢目标后的“最后一次保险包” =====
  bool last_frame_had_control = false;
  float last_sent_yaw = 0.0f;
  float last_sent_pitch = 0.0f;
  bool need_safe_zero_fire_once = false;

  fmt::print("Start High-Performance Loop...\n");

  while (!exiter.exit() && rclcpp::ok()) {
    debug_tool.tick_main_loop();

    need_publish_target_xyz = false;

    // Step 1: 获取前置图像
    if (!front_cam.get(img_front, ts_front)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
    }

    // Step 2: 获取姿态
    Eigen::Quaterniond q;
    {
        ScopedTimer timer(t_q);
        q = gimbal->q(ts_front);
    }
    debug_tool.add_gimbal_q_ms(t_q);

    // Step 3: 前置检测
    std::list<auto_aim::Armor> armors_front;
    {
        ScopedTimer timer(t_yolo);
        solver.set_R_gimbal2world(q);
        armors_front = yolo.detect(img_front);
    }
    debug_tool.add_yolo_detect_ms(t_yolo);

    decider.armor_filter(armors_front);
    decider.set_priority(armors_front);

    // Step 4: 追踪
    auto targets = tracker.track(armors_front, ts_front);
    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    // Step 5: 决策
    if (tracker.state() == "lost") {
        debug_tool.add_lost();

        if (back_cam->get(img_back, ts_back)) {
            auto armors_back = yolo.detect(img_back);
            command = decider.decide_by_armors(armors_back, gimbal_pos, "back");
        } else {
            command = io::Command{false, false, 0, 0};
        }

    } else {
        debug_tool.add_found();
        command = aimer.aim(targets, ts_front, 19.5, false);

        // ===== 新增：准备广播目标坐标 =====
        // 这里取 front 检测结果中优先级最高的装甲板
        // 不依赖 armor.xyz_in_camera 字段，直接由 xyz_in_gimbal 反推回相机系
        if (!armors_front.empty()) {
            const auto & best_armor = armors_front.front();

            publish_xyz_gimbal = best_armor.xyz_in_gimbal;
            publish_xyz_camera =
                solver.R_camera2gimbal().transpose() *
                (publish_xyz_gimbal - solver.t_camera2gimbal());

            need_publish_target_xyz = true;
        }
    }

    // Step 6: 发射与控制
    Eigen::Vector3d ypr = tools::eulers(q.toRotationMatrix(), 2, 1, 0);

    command.shoot = shooter.shoot(command, aimer, targets, ypr);

    // ===== 新增：丢目标后的保险包逻辑 =====
    // 条件：
    // 1) 当前这一帧已经没有控制目标
    // 2) 上一帧是发过控制命令的
    // 则补发一次：保持上次 yaw/pitch，但 shoot = 0
    if (!command.control && last_frame_had_control) {
        need_safe_zero_fire_once = true;
    }

    if (need_safe_zero_fire_once) {
        command.control = true;
        command.shoot = false;
        command.yaw = last_sent_yaw;
        command.pitch = last_sent_pitch;
        need_safe_zero_fire_once = false;
    }

    {
        ScopedTimer timer(t_send);
        if (command.control) {
            gimbal->send(command.control, command.shoot,
                         command.yaw, 0, 0,
                         command.pitch, 0, 0);

            // ===== 新增：发送后同步广播目标坐标 =====
            // 只有当前确实有目标坐标时才广播
            if (need_publish_target_xyz) {
                gimbal->publish_target_xyz(
                    publish_xyz_camera,
                    publish_xyz_gimbal,
                    gimbal->now());
            }

            // ===== 新增：记录最后一次实际发送的 yaw/pitch =====
            last_sent_yaw = command.yaw;
            last_sent_pitch = command.pitch;
            last_frame_had_control = true;
        } else {
            last_frame_had_control = false;
        }
    }
    debug_tool.add_gimbal_send_ms(t_send);

    // Step 7: Debug 报告
    debug_tool.report_if_due(
        tracker.state() == "tracking",
        0,
        5
    );
  }

  rclcpp::shutdown();
  return 0;
}