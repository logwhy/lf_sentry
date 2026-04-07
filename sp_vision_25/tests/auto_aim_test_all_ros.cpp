#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/ros2/gimbal_ros.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

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

  rclcpp::init(argc, argv);

  // -------------------- 模块初始化 --------------------
  io::Camera camera(config_path);
  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto gimbal = std::make_shared<io::GimbalROS>();
  gimbal->start_spin();

  cv::Mat img;

  // -------------------- FPS 统计 --------------------
  double fps = 0.0;
  auto last_fps_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  constexpr double fps_update_interval = 1.0;

  // -------------------- 发送状态缓存 --------------------
  bool last_frame_detected_target = false;  // 上一帧是否检测到目标
  double last_sent_yaw = 0.0;               // 上一次实际发送的 yaw
  double last_sent_pitch = 0.0;             // 上一次实际发送的 pitch

  // -------------------- 主循环 --------------------
  while (!exiter.exit()) {
    auto frame_start = std::chrono::steady_clock::now();

    camera.read(img, frame_start);
    if (img.empty()) {
      break;
    }

    // -------------------- 云台姿态 --------------------
    constexpr auto IMU_DELAY = 2ms;
    Eigen::Quaterniond gimbal_q = gimbal->q(frame_start - IMU_DELAY);
    solver.set_R_gimbal2world(gimbal_q);
    Eigen::Vector3d ypr = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0);

    // -------------------- 自瞄主流程 --------------------
    auto armors = yolo.detect(img, 0);
    auto targets = tracker.track(armors, frame_start);
    auto command = aimer.aim(targets, frame_start, 20, true);
    command.shoot = shooter.shoot(command, aimer, targets, ypr);

    // -------------------- 串口发送 --------------------
    // 这里严格按“当前帧是否检测到装甲板”判断，不用 targets，
    // 因为 tracker 在 temp_lost 时可能仍保留预测目标
    bool detected_target = !armors.empty();

    if (detected_target) {
      // 有目标：正常发送
      gimbal->send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

      // 缓存本次真正发出的角度
      last_sent_yaw = command.yaw;
      last_sent_pitch = command.pitch;
      last_frame_detected_target = true;
    } else if (last_frame_detected_target) {
      // 刚丢失目标：只补发最后一次
      // 要求：不发弹，角度保持上一帧
      gimbal->send(true, false, last_sent_yaw, 0, 0, last_sent_pitch, 0, 0);

      // 让显示 / plotter 也和最后一次实际发送一致
      command.yaw = last_sent_yaw;
      command.pitch = last_sent_pitch;
      command.shoot = false;
      command.control = true;

      // 只补发一次，后续持续无目标时不再发送任何消息
      last_frame_detected_target = false;
    }

    // -------------------- FPS 计算 --------------------
    frame_count++;
    auto now = std::chrono::steady_clock::now();
    double elapsed = tools::delta_time(now, last_fps_time);

    if (elapsed >= fps_update_interval) {
      fps = frame_count / elapsed;
      frame_count = 0;
      last_fps_time = now;

      tools::logger()->info("FPS: {:.1f}, Tracker State: {}", fps, tracker.state());
    }

    // -------------------- 基础文字信息 --------------------
    Eigen::Vector3d euler = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3;

    std::string tracker_state = tracker.state();
    cv::Scalar state_color;
    if (tracker_state == "tracking")
      state_color = {0, 255, 0};
    else if (tracker_state == "temp_lost")
      state_color = {0, 255, 255};
    else if (tracker_state == "detecting")
      state_color = {255, 255, 0};
    else
      state_color = {0, 0, 255};

    tools::draw_text(img, fmt::format("State: {}", tracker_state), {10, 30}, state_color);
    tools::draw_text(
      img, fmt::format("Gimbal yaw:{:.2f}, pitch:{:.2f}", euler[0], euler[2]), {10, 60},
      {255, 255, 255});
    tools::draw_text(
      img, fmt::format("Cmd yaw:{:.2f}, pitch:{:.2f}", command.yaw * 57.3, command.pitch * 57.3),
      {10, 90}, {154, 50, 205});

    if (command.shoot && command.control) {
      tools::draw_text(img, "FIRE", {10, 120}, cv::Scalar(0, 0, 255), 1.2);
    } else {
      tools::draw_text(img, "HOLD", {10, 120}, cv::Scalar(200, 200, 200));
    }

    // for (const auto & target : targets) {
    //   const auto & armor_xyza_list = target.armor_xyza_list();

    //   for (size_t i = 0; i < armor_xyza_list.size(); ++i) {
    //     const Eigen::Vector4d & xyza = armor_xyza_list[i];

    //     // 1. 画装甲板（绿色）
    //     auto image_points =
    //       solver.reproject_armor(xyza.head<3>(), xyza[3], target.armor_type, target.name);
    //     tools::draw_points(img, image_points, {0, 255, 0});

    //     // 2. 装甲板中心 → 像素坐标
    //     std::vector<cv::Point3f> world_pts = {cv::Point3f(
    //       static_cast<float>(xyza[0]), static_cast<float>(xyza[1]), static_cast<float>(xyza[2]))};

    //     auto pixel_pts = solver.world2pixel(world_pts);
    //     if (pixel_pts.empty()) continue;

    //     cv::Point2f center_pt = pixel_pts[0];

    //     // 3. 画 id（红字）
    //     cv::putText(
    //       img, fmt::format("id={}", i), center_pt + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX,
    //       0.6, cv::Scalar(0, 0, 255), 2);
    //   }
    // }

    // // 绘制瞄准点
    // if (!targets.empty() && aimer.debug_aim_point.valid) {
    //   Eigen::Vector3d aim_xyz(
    //     aimer.debug_aim_point.xyza[0], aimer.debug_aim_point.xyza[1],
    //     aimer.debug_aim_point.xyza[2]);

    //   std::vector<cv::Point3f> world_pts = {cv::Point3f(
    //     static_cast<float>(aim_xyz.x()), static_cast<float>(aim_xyz.y()),
    //     static_cast<float>(aim_xyz.z()))};

    //   // 投影到图像平面
    //   auto pixel_pts = solver.world2pixel(world_pts);

    //   if (!pixel_pts.empty()) {
    //     cv::Point2f pt = pixel_pts[0];
    //     // 实心红圆 + 白边（更醒目）
    //     cv::circle(img, pt, 5, cv::Scalar(0, 0, 255), -1);     // 红色填充
    //     cv::circle(img, pt, 6, cv::Scalar(255, 255, 255), 1);  // 白色轮廓
    //   }
    // }

    // ==================== 【End 可视化】====================

    // -------------------- Plotter --------------------
    nlohmann::json data;
    if (!targets.empty()) {
      auto target = targets.front();
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;
      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }
    data["fps"] = fps;
    data["gimbal_yaw"] = euler[0];
    data["gimbal_pitch"] = euler[2];
    data["cmd_yaw"] = command.yaw * 57.3;
    data["cmd_pitch"] = command.pitch * 57.3;
    data["track_state"] = tracker_state;
    data["control"] = command.control;
    data["shoot"] = command.shoot;

    plotter.plot(data);

    // -------------------- 显示 --------------------
    cv::resize(img, img, {}, 0.01, 0.01);
    cv::imshow("auto_aim", img);
    if (cv::waitKey(1) == 'q') {
      break;
    }
  }

  rclcpp::shutdown();
  return 0;
}