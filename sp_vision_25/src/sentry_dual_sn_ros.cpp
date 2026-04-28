#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <list>
#include <memory>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "async_detect_worker.hpp"
#include "io/command.hpp"
#include "io/ros2/gimbal_ros.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/debug_tool.hpp"
#include "tools/exiter.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明}"
  "{@config-path   | configs/demo.yaml | demo.yaml 配置文件路径}";

static void overlay_if_exists(YAML::Node & dst, const YAML::Node & src, const std::string & key)
{
  if (src[key]) dst[key] = src[key];
}

static std::string read_config_path(
  const YAML::Node & root,
  const std::string & key,
  const std::string & fallback)
{
  return root[key] ? root[key].as<std::string>() : fallback;
}

static YAML::Node build_runtime_config(
  const std::string & demo_path,
  const std::string & cam_path,
  bool is_back_camera)
{
  YAML::Node demo = YAML::LoadFile(demo_path);
  YAML::Node cam = YAML::LoadFile(cam_path);
  YAML::Node merged = YAML::Clone(demo);

  // 相机打开相关、标定相关从 cam1/cam2 继承；
  // 模型、enemy_color、阈值、tracker/aimer/shooter 策略默认跟随 demo。
  const char * overlay_keys[] = {
    "camera_name", "serial_number", "vid_pid", "exposure_ms", "gain", "gamma",
    "image_width", "image_height", "camera_matrix", "distort_coeffs",
    "R_camera2gimbal", "t_camera2gimbal", "R_gimbal2imubody"};

  for (const auto * k : overlay_keys) overlay_if_exists(merged, cam, k);

  // device 优先级：
  // 前摄：front_device > demo.device > cam.device > GPU
  // 后摄：cam.device > back_device_default > demo.device > NPU
  if (!is_back_camera) {
    if (demo["front_device"]) merged["device"] = demo["front_device"];
    else if (demo["device"]) merged["device"] = demo["device"];
    else if (cam["device"]) merged["device"] = cam["device"];
    else merged["device"] = "GPU";
  } else {
    if (cam["device"]) merged["device"] = cam["device"];
    else if (demo["back_device_default"]) merged["device"] = demo["back_device_default"];
    else if (demo["device"]) merged["device"] = demo["device"];
    else merged["device"] = "NPU";
  }

  return merged;
}

static void save_yaml(const YAML::Node & node, const std::string & path)
{
  std::ofstream fout(path);
  if (!fout.is_open()) throw std::runtime_error("failed to open " + path);
  fout << node;
}

static Eigen::Vector3d xyz_camera_from_gimbal(
  const auto_aim::Solver & solver,
  const Eigen::Vector3d & xyz_gimbal)
{
  return solver.R_camera2gimbal().transpose() * (xyz_gimbal - solver.t_camera2gimbal());
}

static bool is_fresh(const sp_vision::DetectPacket & packet, double max_age_ms)
{
  const double age_ms =
    std::chrono::duration<double, std::milli>(SteadyClock::now() - packet.ts).count();
  return age_ms <= max_age_ms;
}

static void sort_by_priority(std::list<auto_aim::Armor> & armors)
{
  armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) {
    return a.priority < b.priority;
  });
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  tools::Exiter exiter;
  tools::Recorder recorder;
  tools::DebugTool debug(2);

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    rclcpp::shutdown();
    return 0;
  }

  const std::string demo_path = cli.get<std::string>(0);
  YAML::Node demo = YAML::LoadFile(demo_path);

  const std::string cam1_path = read_config_path(demo, "front_camera_config", "configs/demo.yaml");
  const std::string cam2_path = read_config_path(demo, "back_camera_config", "configs/cam2.yaml");

  const std::string front_runtime = "/tmp/sp_front_runtime.yaml";
  const std::string back_runtime = "/tmp/sp_back_runtime.yaml";
  save_yaml(build_runtime_config(demo_path, cam1_path, false), front_runtime);
  save_yaml(build_runtime_config(demo_path, cam2_path, true), back_runtime);

  YAML::Node front_yaml = YAML::LoadFile(front_runtime);
  YAML::Node back_yaml = YAML::LoadFile(back_runtime);

  fmt::print("[INFO] Front camera config: {}\n", cam1_path);
  fmt::print("[INFO] Back  camera config: {}\n", cam2_path);
  fmt::print("[INFO] Front runtime: {}\n", front_runtime);
  fmt::print("[INFO] Back  runtime: {}\n", back_runtime);
  fmt::print("[INFO] Front device: {}\n", front_yaml["device"].as<std::string>());
  fmt::print("[INFO] Back  device: {}\n", back_yaml["device"].as<std::string>());
  fmt::print("[INFO] Enemy color: {}\n", front_yaml["enemy_color"].as<std::string>());

  auto gimbal = std::make_shared<io::GimbalROS>();
  gimbal->start_spin();

  // 相机启动向 sentry_dual_cameras_ros 看齐：
  // 1) 明确使用 configs/cam1.yaml / configs/cam2.yaml；
  // 2) 明确使用 io::SNCamera；
  // 3) 两个相机打开之间保留 2s 间隔，避免 SDK/USB 枚举抢占。
  fmt::print("[INFO] Opening SNCamera streams...\n");
  sp_vision::AsyncDetectWorker front_worker("Front", cam1_path, &debug, 0);
  std::this_thread::sleep_for(2s);
  sp_vision::AsyncDetectWorker back_worker("Back", cam2_path, &debug, 1);

  // 两台相机都打开后，再初始化 YOLO 并启动检测线程。
  // 这样不会出现“前摄模型初始化很久，后摄还没来得及打开”的问题。
  fmt::print("[INFO] Starting async detect workers...\n");
  front_worker.start(front_runtime);
  back_worker.start(back_runtime);

  auto_aim::Solver solver_front(front_runtime);
  auto_aim::Solver solver_back(back_runtime);
  auto_aim::Tracker tracker(front_runtime, solver_front);
  auto_aim::Aimer aimer(front_runtime);
  auto_aim::Shooter shooter(front_runtime);
  omniperception::Decider decider(front_runtime);

  uint64_t last_front_seq = 0;
  bool last_frame_had_control = false;
  float last_sent_yaw = 0.0f;
  float last_sent_pitch = 0.0f;

  double t_q = 0.0;
  double t_send = 0.0;

  fmt::print("[INFO] Start async dual-camera ROS loop.\n");

  while (!exiter.exit() && rclcpp::ok()) {
    debug.tick_main_loop();

    // 主循环由前摄检测结果驱动，节奏尽量接近 auto_aim_test_all_ros 的单相机主链路。
    sp_vision::DetectPacket front;
    if (!front_worker.latest(front) || !front.valid || front.seq == last_front_seq) {
      std::this_thread::sleep_for(1ms);
      continue;
    }
    last_front_seq = front.seq;

    Eigen::Quaterniond q_front;
    {
      sp_vision::ScopedTimer timer(t_q);
      q_front = gimbal->q(front.ts - 2ms);
    }
    debug.add_gimbal_q_ms(t_q);

    solver_front.set_R_gimbal2world(q_front);
    Eigen::Vector3d ypr = tools::eulers(q_front.toRotationMatrix(), 2, 1, 0);

    auto armors_front = front.armors;
    decider.armor_filter(armors_front);
    decider.set_priority(armors_front);
    sort_by_priority(armors_front);

    auto targets = tracker.track(armors_front, front.ts);

    io::Command command{false, false, 0, 0};
    bool need_publish_xyz = false;
    Eigen::Vector3d xyz_camera = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz_gimbal = Eigen::Vector3d::Zero();

    if (tracker.state() != "lost") {
      debug.add_found();

      command = aimer.aim(targets, front.ts, 20, true);
      command.shoot = shooter.shoot(command, aimer, targets, ypr);

      if (command.control && !armors_front.empty()) {
        const auto & best = armors_front.front();
        xyz_gimbal = best.xyz_in_gimbal;
        xyz_camera = xyz_camera_from_gimbal(solver_front, xyz_gimbal);
        need_publish_xyz = true;
      }
    } else {
      debug.add_lost();

      sp_vision::DetectPacket back;
      if (back_worker.latest(back) && back.valid && is_fresh(back, 120.0)) {
        Eigen::Quaterniond q_back = gimbal->q(back.ts - 2ms);
        solver_back.set_R_gimbal2world(q_back);
        const Eigen::Vector3d gimbal_pos_back =
          tools::eulers(solver_back.R_gimbal2world(), 2, 1, 0);

        auto armors_back = back.armors;
        decider.armor_filter(armors_back);
        decider.set_priority(armors_back);
        sort_by_priority(armors_back);

        command = decider.decide_by_armors(armors_back, gimbal_pos_back, "back");
        command.shoot = false;  // 后摄只辅助转向，不允许直接开火。

        if (command.control && !armors_back.empty()) {
          const auto & best = armors_back.front();
          xyz_gimbal = best.xyz_in_gimbal;
          xyz_camera = xyz_camera_from_gimbal(solver_back, xyz_gimbal);
          need_publish_xyz = true;
        }
      }
    }

    // 对齐 auto_aim_test_all_ros：刚丢目标时只补发一次上一帧角度，不持续刷旧命令。
    if (!command.control && last_frame_had_control) {
      command.control = true;
      command.shoot = false;
      command.yaw = last_sent_yaw;
      command.pitch = last_sent_pitch;
      need_publish_xyz = false;
      last_frame_had_control = false;
    } else if (!command.control) {
      last_frame_had_control = false;
    }

    {
      sp_vision::ScopedTimer timer(t_send);
      if (command.control) {
        gimbal->send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

        if (need_publish_xyz) {
          gimbal->publish_target_xyz(xyz_camera, xyz_gimbal, gimbal->now());
        }

        last_sent_yaw = command.yaw;
        last_sent_pitch = command.pitch;
        last_frame_had_control = true;
      }
    }
    debug.add_gimbal_send_ms(t_send);

    debug.report_if_due(tracker.state() == "tracking", 0, 5);
  }

  front_worker.stop();
  back_worker.stop();
  gimbal->stop_spin();
  rclcpp::shutdown();
  return 0;
}