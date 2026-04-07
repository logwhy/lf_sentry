#include <fmt/core.h>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"  // ✅ 多线程检测模块
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{@config-path   | configs/demo.yaml | yaml配置文件路径}";

int main(int argc, char * argv[])
{
  // -------------------- 参数读取 --------------------
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;
  tools::Plotter plotter;

  // -------------------- 模块初始化 --------------------
  io::Camera camera(config_path);        // 工业相机
  io::Gimbal gimbal(config_path);        // 串口通信对象
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::multithread::MultiThreadDetector detector(config_path);  // ✅ YOLO异步检测器

  // -------------------- 图像采集线程 --------------------
  std::thread detect_thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;

    while (!exiter.exit()) {
      camera.read(img, timestamp);
      if (img.empty()) continue;
      detector.push(img, timestamp);  // 推入检测队列
    }
  });

  // -------------------- 主线程（控制与可视化） --------------------
  // ==================== 添加抗丢帧变量 ====================
  io::Command last_valid_command{};      // 保存最后一次有效命令
  int consecutive_lost_frames = 0;       // 连续丢失帧数
  constexpr int MAX_LOST_FRAMES = 8;     // 最大允许丢失帧数
  
  // 原有的变量
  io::Command last_command{};
  double last_t = -1;
  double fps = 0.0;
  auto last_frame_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  constexpr double fps_update_interval = 1.0;

  while (!exiter.exit()) {
    // 从检测线程获取图像和装甲板检测结果
    auto [img, armors, timestamp] = detector.debug_pop();
    if (img.empty()) continue;

    // 读取云台姿态四元数（来自下位机）
    Eigen::Quaterniond gimbal_q = gimbal.q(timestamp);
    solver.set_R_gimbal2world(gimbal_q);

    // -------------------- 自瞄核心 --------------------
    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, 27, false);

    // ==================== 添加抗丢帧逻辑 ====================
    std::string tracker_state = tracker.state();
    bool should_control = (tracker_state != "lost");

    if (should_control) {
      // 跟踪器认为应该控制（包括预测状态）
      command.control = true;
      
      // 检查角度有效性
      if (std::isfinite(command.yaw) && std::isfinite(command.pitch)) {
        last_valid_command = command;
        consecutive_lost_frames = 0;
        
        // 更新last_command用于射击判断
        last_command = command;
      } else {
        // 角度无效，使用历史命令
        command = last_valid_command;
        tools::logger()->warn("Invalid angles, using last valid command");
      }
    } else {
      // 跟踪器认为完全丢失
      consecutive_lost_frames++;
      
      if (consecutive_lost_frames <= MAX_LOST_FRAMES) {
        // 使用历史命令继续控制一段时间
        command = last_valid_command;
        command.control = true;
        tools::logger()->info("Target lost {}/{}, using prediction", 
                             consecutive_lost_frames, MAX_LOST_FRAMES);
      } else {
        // 完全停止
        command.control = false;
        command.yaw = 0;
        command.pitch = 0;
        tools::logger()->warn("Target completely lost, stopping control");
      }
    }

    // 连续帧目标平稳时自动开火（只在有效跟踪时）
    if (tracker_state == "tracking" && !targets.empty() && aimer.debug_aim_point.valid &&
        std::abs(command.yaw - last_command.yaw) * 57.3 < 2) {
      command.shoot = true;
    } else {
      command.shoot = false;  // 确保在预测或丢失时不射击
    }

    // -------------------- 串口发送 --------------------
    if (command.control) {
      // 只在命令有效时更新last_command（用于射击判断）
      last_command = command;
      
      gimbal.send(command.control, command.shoot,
                  command.yaw, 0, 0,
                  command.pitch, 0, 0);

      tools::logger()->info(
        "Send cmd - yaw: {:.2f}°, pitch: {:.2f}°, shoot: {}, state: {}, lost: {}/{}",
        command.yaw * 57.3, command.pitch * 57.3, command.shoot, tracker_state,
        consecutive_lost_frames, MAX_LOST_FRAMES);
    } else {
      // 发送停止命令
      gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
      tools::logger()->info("No control - state: {}, lost: {}/{}", 
                           tracker_state, consecutive_lost_frames, MAX_LOST_FRAMES);
    }

    // -------------------- 运行时信息 --------------------
    auto finish = std::chrono::steady_clock::now();
    
    // 减少日志频率避免影响性能
    static int log_counter = 0;
    if (log_counter++ % 5 == 0) {
      tools::logger()->info(
        "tracker: {:.1f}ms, aimer: {:.1f}ms, state: {}",
        tools::delta_time(aimer_start, tracker_start) * 1e3,
        tools::delta_time(finish, aimer_start) * 1e3, tracker_state);
    }

    // 当前云台姿态角
    Eigen::Vector3d euler = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3;

    // ==================== 增强的可视化 ====================
    // 显示跟踪状态（不同颜色表示不同状态）
    cv::Scalar state_color;
    if (tracker_state == "tracking") state_color = {0, 255, 0};        // 绿色：跟踪中
    else if (tracker_state == "temp_lost") state_color = {0, 255, 255}; // 黄色：预测中
    else if (tracker_state == "detecting") state_color = {255, 255, 0}; // 青色：检测中
    else state_color = {0, 0, 255};                                    // 红色：丢失
    
    tools::draw_text(img, 
                     fmt::format("State: {} [{}/{}]", tracker_state, consecutive_lost_frames, MAX_LOST_FRAMES),
                     {10, 30}, state_color);
    
    tools::draw_text(img, 
                     fmt::format("gimbal yaw:{:.2f}, pitch:{:.2f}", euler[0], euler[1]), 
                     {10, 60}, {255, 255, 255});
    tools::draw_text(img, 
                     fmt::format("cmd yaw:{:.2f}, pitch:{:.2f}, shoot:{}",
                                command.yaw * 57.3, command.pitch * 57.3, command.shoot),
                     {10, 90}, {154, 50, 205});

    // 绘制 aimer 瞄准点
    if (!targets.empty() && aimer.debug_aim_point.valid) {
      auto &aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto &target = targets.front();
      auto image_points =
        solver.reproject_armor(aim_xyza.head<3>(), aim_xyza[3],
                               target.armor_type, target.name);
      
      // 根据跟踪状态改变颜色
      cv::Scalar aim_color = (tracker_state == "tracking") ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
      tools::draw_points(img, image_points, aim_color);
    }
    
    // 在预测状态显示提示
    if (tracker_state == "temp_lost") {
      tools::draw_text(img, "PREDICTING...", {10, 120}, {255, 255, 0});
    }

    // 绘图数据
    nlohmann::json data;
    data["gimbal_yaw"] = euler[0];
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;
    data["track_state"] = tracker_state;
    data["lost_frames"] = consecutive_lost_frames;
    data["control"] = command.control;
    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("reprojection", img);
    if (cv::waitKey(1) == 'q') break;

    // -------------------- FPS计算 --------------------
    frame_count++;
    double elapsed = tools::delta_time(std::chrono::steady_clock::now(), last_frame_time);
    if (elapsed >= fps_update_interval) {
      fps = frame_count / elapsed;
      frame_count = 0;
      last_frame_time = std::chrono::steady_clock::now();
      tools::logger()->info("Current FPS: {:.1f}, State: {}", fps, tracker_state);
    }
  }

  // -------------------- 线程退出 --------------------
  detect_thread.join();
  return 0;
}