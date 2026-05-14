#include <fmt/core.h>

#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>

#include "io/camera.hpp"
#include "io/ros2/gimbal_ros.hpp"

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"

#include "tools/exiter.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

// ==========================================
// 相机抓取流 (Producer)
// ==========================================
struct FrameData {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
};

class CameraStream {
public:
    CameraStream(io::SNCamera* cam) : camera_(cam), running_(true) {
        thread_ = std::thread(&CameraStream::grab_loop, this);
    }
    ~CameraStream() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }
    bool get_latest(cv::Mat& out_img, std::chrono::steady_clock::time_point& out_t) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_frame_.img.empty()) return false;
        
        // 【修改点 1】：将 copyTo 改为 swap，实现图像零拷贝，极大节省 Intel NUC 的内存带宽
        cv::swap(out_img, latest_frame_.img); 
        out_t = latest_frame_.timestamp;
        return true;
    }
private:
    void grab_loop() {
        while (running_) {
            cv::Mat img;
            auto t = std::chrono::steady_clock::now();
            camera_->read(img, t); 
            if (!img.empty()) {
                std::lock_guard<std::mutex> lock(mutex_);
                // 【修改点 1 配合】：同样使用 swap 移交图像所有权
                cv::swap(latest_frame_.img, img);
                latest_frame_.timestamp = t;
            }
        }
    }
    io::SNCamera* camera_;
    std::atomic<bool> running_;
    std::thread thread_;
    std::mutex mutex_;
    FrameData latest_frame_;
};

// ==========================================
// 线程间共享数据与锁
// ==========================================
std::mutex cmd_mtx;

bool front_locked = false;
io::Command front_cmd{false, false, 0, 0};

// ===== 新增：只缓存前摄当前画面给出的 xyz =====
bool front_xyz_valid = false;
Eigen::Vector3d front_xyz_camera = Eigen::Vector3d::Zero();
Eigen::Vector3d front_xyz_gimbal = Eigen::Vector3d::Zero();

bool back_has_target = false;
io::Command back_cmd{false, false, 0, 0};

std::atomic<bool> req_clear_front{false}; // 通知前摄线程清空缓存
std::atomic<int> fps_counter{0};          // 仅前摄帧率

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/demo.yaml      | 位置参数，yaml配置文件路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) { cli.printMessage(); return 0; }
  auto config_path = cli.get<std::string>(0);
  rclcpp::init(argc, argv);

  tools::Exiter exiter;
  auto gimbal = std::make_shared<io::GimbalROS>();
  gimbal->start_spin();

  // 1. 初始化相机与流
  io::SNCamera camera(config_path);
  std::this_thread::sleep_for(2s);
  io::SNCamera back_camera("configs/cam2.yaml");

  CameraStream front_stream(&camera);
  CameraStream back_stream(&back_camera);

  // 2. 初始化核心模块
  auto_aim::YOLO yolo_front(config_path, false);
  auto_aim::YOLO yolo_back(config_path, false);
  
  auto_aim::Solver solver(config_path);
  auto tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  omniperception::Decider decider(config_path);

  constexpr auto IMU_DELAY = 2ms;
  constexpr int DROP_FRAMES_AFTER_BACK = 3;

  // ==========================================
  // 前摄推理线程 (Consumer 1) - 满载运行
  // ==========================================
  std::thread front_worker([&]() {
      cv::Mat img;
      int drop_frames = 0;
      
      while (!exiter.exit()) {
          std::chrono::steady_clock::time_point t;
          if (!front_stream.get_latest(img, t)) {
              std::this_thread::sleep_for(1ms);
              continue;
          }

          if (req_clear_front.exchange(false)) {
              tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
              drop_frames = DROP_FRAMES_AFTER_BACK;
          }

          Eigen::Quaterniond q = gimbal->q(t - IMU_DELAY);
          solver.set_R_gimbal2world(q);
          Eigen::Vector3d pos = tools::eulers(q.toRotationMatrix(), 2, 1, 0);

          auto armors = yolo_front.detect(img, 0);
          if (drop_frames > 0) {
              armors.clear();
              drop_frames--;
          }

          decider.armor_filter(armors);
          decider.set_priority(armors);
          auto targets = tracker->track(armors, t);

          io::Command cmd{false, false, 0, 0};
          bool locked = (tracker->state() != "lost");

          bool xyz_valid = false;
          Eigen::Vector3d xyz_camera = Eigen::Vector3d::Zero();
          Eigen::Vector3d xyz_gimbal = Eigen::Vector3d::Zero();

          if (locked && !armors.empty()) {
              const auto & best_armor = armors.front();
              xyz_gimbal = best_armor.xyz_in_gimbal;
              xyz_camera =
                  solver.R_camera2gimbal().transpose() *
                  (xyz_gimbal - solver.t_camera2gimbal());
              xyz_valid = true;
          }

          if (locked) {
              const auto bullet_speed = static_cast<double>(gimbal->state().bullet_speed);
              cmd = aimer.aim(targets, t, bullet_speed, true);
              cmd.shoot = shooter.shoot(cmd, aimer, targets, pos);
          }

          {
              std::lock_guard<std::mutex> lock(cmd_mtx);
              front_locked = locked;
              front_cmd = cmd;

              front_xyz_valid = xyz_valid;
              front_xyz_camera = xyz_camera;
              front_xyz_gimbal = xyz_gimbal;
          }
          fps_counter++;
      }
  });

  // ==========================================
  // 后摄推理线程 (Consumer 2) - 限制 60 FPS
  // ==========================================
  std::thread back_worker([&]() {
      cv::Mat img;
      // 【修改点 2】：设定 60 帧的最小周期 (1000ms / 60 ≈ 16.67ms = 16667us)
      const auto min_period = std::chrono::microseconds(16667); 

      while (!exiter.exit()) {
          auto start_time = std::chrono::steady_clock::now();

          std::chrono::steady_clock::time_point t;
          if (!back_stream.get_latest(img, t)) {
              std::this_thread::sleep_for(1ms); // 原来的 2ms 改为 1ms，防止因为等待导致错过 60 帧对齐
              continue;
          }

          Eigen::Quaterniond q = gimbal->q(t - IMU_DELAY);
          Eigen::Vector3d pos = tools::eulers(q.toRotationMatrix(), 2, 1, 0);

          auto armors = yolo_back.detect(img, 0);
          
          io::Command cmd{false, false, 0, 0};
          bool has_target = false;

          if (!armors.empty()) {
              cmd = decider.decide_by_armors(armors, pos, "back");
              if (cmd.control) has_target = true;
          }

          {
              std::lock_guard<std::mutex> lock(cmd_mtx);
              back_has_target = has_target;
              back_cmd = cmd;
          }

          // 【修改点 2】：帧率限制逻辑，保证不低于 60 帧，但绝不超跑
          auto end_time = std::chrono::steady_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
          if (elapsed < min_period) {
              std::this_thread::sleep_for(min_period - elapsed);
          }
      }
  });

  // ==========================================
  // 主控线程 (Controller / State Machine)
  // ==========================================
  bool waiting_front_lock = false;
  auto wait_until = std::chrono::steady_clock::now();
  auto cooldown_until = std::chrono::steady_clock::now();
  
  double last_yaw = 0.0, last_pitch = 0.0;
  bool last_front_detected = false;
  auto last_fps_time = std::chrono::steady_clock::now();

  double back_target_yaw = 0.0; 
  double back_target_pitch = 0.0;

  while (!exiter.exit()) {
      auto now = std::chrono::steady_clock::now();

      bool cur_f_locked = false;
      io::Command cur_f_cmd{false, false, 0, 0};

      bool cur_b_has_target = false;
      io::Command cur_b_cmd{false, false, 0, 0};

      bool cur_f_xyz_valid = false;
      Eigen::Vector3d cur_f_xyz_camera = Eigen::Vector3d::Zero();
      Eigen::Vector3d cur_f_xyz_gimbal = Eigen::Vector3d::Zero();

      {
          std::lock_guard<std::mutex> lock(cmd_mtx);
          cur_f_locked = front_locked;
          cur_f_cmd = front_cmd;
          cur_b_has_target = back_has_target;
          cur_b_cmd = back_cmd;

          cur_f_xyz_valid = front_xyz_valid;
          cur_f_xyz_camera = front_xyz_camera;
          cur_f_xyz_gimbal = front_xyz_gimbal;
      }

      // --- 状态机仲裁与发送 ---
      if (cur_f_locked) {
        waiting_front_lock = false;
        cooldown_until = now + 2000ms; 
        
        gimbal->send(true, cur_f_cmd.shoot, cur_f_cmd.yaw, 0, 0, cur_f_cmd.pitch, 0, 0);

        if (cur_f_xyz_valid) {
            gimbal->publish_target_xyz(
                cur_f_xyz_camera,
                cur_f_xyz_gimbal,
                gimbal->now());
        }

        last_yaw = cur_f_cmd.yaw;
        last_pitch = cur_f_cmd.pitch;
        last_front_detected = true;
      } 
      else {
          if (waiting_front_lock && now < wait_until) {
              gimbal->send(true, false, back_target_yaw, 0, 0, back_target_pitch, 0, 0);
          } 
          else if (now > cooldown_until && cur_b_has_target) {
              back_target_yaw = cur_b_cmd.yaw;
              back_target_pitch = cur_b_cmd.pitch;
              
              gimbal->send(true, false, back_target_yaw, 0, 0, back_target_pitch, 0, 0);
              
              waiting_front_lock = true;
              wait_until = now + 800ms;
              cooldown_until = now + 2000ms; 
              
              req_clear_front = true; 
              last_front_detected = false;
          }
          else {
              if (last_front_detected) {
                  gimbal->send(true, false, last_yaw, 0, 0, last_pitch, 0, 0);
                  last_front_detected = false;
              } 
          }
      }

      // --- FPS 极简刷新 ---
      double elapsed = tools::delta_time(now, last_fps_time);
      if (elapsed >= 1.0) {
          int frames = fps_counter.exchange(0);
          fmt::print("\r[Parallel Vision] F_FPS: {:.1f} | F_Lock: {} | B_Target: {}", 
                     frames / elapsed, cur_f_locked, cur_b_has_target);
          fflush(stdout);
          last_fps_time = now;
      }

      // 【修改点 3】：极短休眠让出 CPU 切片，彻底解决主控线程 100% 占用问题
      std::this_thread::sleep_for(1ms); 
  }

  front_worker.join();
  back_worker.join();
  
  fmt::print("\n");
  rclcpp::shutdown();
  return 0;
}
