#include <fmt/core.h>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
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

  tools::Plotter plotter;
  tools::Exiter exiter;

  // -------------------- 模块初始化 --------------------
  io::Camera camera(config_path);  //  工业相机
  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  io::Gimbal gimbal(config_path);  //  串口通信对象

  cv::Mat img;
  auto_aim::Target last_target;
  
  // ==================== 添加防丢帧变量 ====================
  io::Command last_valid_command{};           // 保存最后一次有效命令
  int consecutive_lost_frames = 0;            // 连续丢失帧数
  constexpr int MAX_LOST_FRAMES = 8;          // 最大允许丢失帧数
  bool target_stable = false;                 // 目标稳定标志
  int stable_frame_count = 0;                 // 稳定帧计数
  
  double last_t = -1;
  std::chrono::steady_clock::time_point timestamp;

  double fps = 0.0;
  auto last_frame_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  constexpr double fps_update_interval = 1.0;

  // -------------------- 主循环 --------------------
  while (!exiter.exit()) {
    auto frame_start = std::chrono::steady_clock::now();

    camera.read(img, timestamp);  //  从工业相机读取图像和时间戳

    if (img.empty()) break;

    //  从下位机读取云台姿态四元数
    Eigen::Quaterniond gimbal_q = gimbal.q(timestamp);
    solver.set_R_gimbal2world(gimbal_q);

    // -------------------- 自瞄核心 --------------------
    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, 0);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, 27, false);

    // ==================== 添加防丢帧逻辑 ====================
    std::string tracker_state = tracker.state();
    
    // 更新目标稳定性
    bool has_valid_target = !targets.empty() && command.control;
    if (has_valid_target) {
      consecutive_lost_frames = 0;
      stable_frame_count++;
      
      // 连续3帧检测到目标认为稳定
      if (stable_frame_count >= 3) {
        target_stable = true;
      }
      
      // 更新有效命令
      if (std::isfinite(command.yaw) && std::isfinite(command.pitch)) {
        last_valid_command = command;
      }
    } else {
      consecutive_lost_frames++;
      stable_frame_count = 0;
      target_stable = false;
    }
    
    // 根据跟踪器状态和丢失情况调整命令
    bool should_control = (tracker_state != "lost");
    
    if (should_control) {
      // 跟踪器认为应该控制（包括预测状态）
      command.control = true;
      
      // 如果当前命令无效，使用历史命令
      if (!std::isfinite(command.yaw) || !std::isfinite(command.pitch)) {
        command = last_valid_command;
        tools::logger()->warn("Invalid command, using last valid command");
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

    // 连续帧目标平稳时自动开火（只在稳定跟踪时）
    if (target_stable && !targets.empty() && aimer.debug_aim_point.valid &&
        std::abs(command.yaw - last_valid_command.yaw) * 57.3 < 2) {
      command.shoot = true;
    } else {
      command.shoot = false;  // 确保在预测或丢失时不射击
    }

    // -------------------- 串口发送控制命令 --------------------
    if (command.control) {
      // 更新用于射击判断的last_command
      last_valid_command = command;
      
      tools::logger()->info(
        "Sending command - yaw: {:.2f}°, pitch: {:.2f}°, shoot: {}, state: {}, lost: {}/{}", 
        command.yaw * 57.3, command.pitch * 57.3, command.shoot, tracker_state,
        consecutive_lost_frames, MAX_LOST_FRAMES);
    } else {
      tools::logger()->info("No control - state: {}, lost: {}/{}", 
                           tracker_state, consecutive_lost_frames, MAX_LOST_FRAMES);
    }

    gimbal.send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

    // -------------------- 帧率计算 --------------------
    auto frame_end = std::chrono::steady_clock::now();
    double frame_time = tools::delta_time(frame_end, frame_start);

    // 更新帧率计数器
    frame_count++;
    double elapsed_time = tools::delta_time(frame_end, last_frame_time);

    // 每秒更新一次FPS
    if (elapsed_time >= fps_update_interval) {
      fps = frame_count / elapsed_time;
      frame_count = 0;
      last_frame_time = frame_end;

      // 输出当前帧率
      tools::logger()->info("Current FPS: {:.1f}, State: {}", fps, tracker_state);
    }

    // ==================== 增强的可视化 ====================
    auto finish = std::chrono::steady_clock::now();

    // 减少性能日志频率避免影响性能
    static int perf_log_counter = 0;
    if (perf_log_counter++ % 5 == 0) {
      tools::logger()->info(
        "yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms",
        tools::delta_time(tracker_start, yolo_start) * 1e3,
        tools::delta_time(aimer_start, tracker_start) * 1e3,
        tools::delta_time(finish, aimer_start) * 1e3);
    }

    Eigen::Vector3d euler = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3;
    
    // 显示跟踪状态（不同颜色）
    cv::Scalar state_color;
    if (tracker_state == "tracking") state_color = {0, 255, 0};        // 绿色
    else if (tracker_state == "temp_lost") state_color = {0, 255, 255}; // 黄色
    else if (tracker_state == "detecting") state_color = {255, 255, 0}; // 青色
    else state_color = {0, 0, 255};                                    // 红色
    
    tools::draw_text(
      img, 
      fmt::format("State: {} [{}/{}]", tracker_state, consecutive_lost_frames, MAX_LOST_FRAMES), 
      {10, 30}, state_color);
    
    tools::draw_text(
      img, fmt::format("gimbal yaw:{:.2f}, pitch:{:.2f}", euler[0], euler[2]), {10, 60},
      {255, 255, 255});
    tools::draw_text(
      img,
      fmt::format(
        "cmd yaw:{:.2f}, pitch:{:.2f}, shoot:{}", command.yaw * 57.3, command.pitch * 57.3,
        command.shoot),
      {10, 90}, {154, 50, 205});
      
    // 显示目标稳定性
    if (target_stable) {
      tools::draw_text(img, "STABLE", {10, 120}, {0, 255, 0});
    } else if (has_valid_target) {
      tools::draw_text(img, "ACQUIRING", {10, 120}, {0, 255, 255});
    }

    // -------------------- 绘制 debug aim point --------------------
    if (!targets.empty() && aimer.debug_aim_point.valid) {
      auto & aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza;
      for (int i = 0; i < 4; ++i) {
        aim_xyza[i] = static_cast<double>(aim_point.xyza[i]);
      }

      auto & first_target = targets.front();
      auto image_points = solver.reproject_armor(
        aim_xyza.head<3>(), aim_xyza[3], first_target.armor_type, first_target.name);
        
      // 根据稳定性改变颜色
      cv::Scalar point_color = target_stable ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
      tools::draw_points(img, image_points, point_color);
    }
    
    // 在预测状态显示提示
    if (tracker_state == "temp_lost") {
      tools::draw_text(img, "PREDICTING...", {10, 150}, {255, 255, 0});
    }

    // -------------------- 观测与绘图 --------------------
    nlohmann::json data;
    data["gimbal_yaw"] = euler[0];
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;
    data["gimbal_pitch"] = euler[2];
    data["cmd_pitch"] = command.pitch * 57.3;
    data["track_state"] = tracker_state;
    data["lost_frames"] = consecutive_lost_frames;
    data["stable"] = target_stable;
    data["control"] = command.control;

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("reprojection", img);
    if (cv::waitKey(1) == 'q') break;
  }
  
  return 0;
}
