#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

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

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/demo.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto gimbal = std::make_shared<io::GimbalROS>();
  gimbal->start_spin(); 
  io::SNCamera camera("configs/cam1.yaml");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  io::SNCamera back_camera("configs/cam2.yaml");

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);

  cv::Mat img;

  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q;
    
    q = gimbal->q(timestamp);

    // recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);

    // decider.get_invincible_armor(ros2.subscribe_enemy_status());

    decider.armor_filter(armors);

    // decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());

    decider.set_priority(armors);

    auto targets = tracker.track(armors, timestamp);

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
    if (tracker.state() == "lost")
      command = decider.decide(yolo, gimbal_pos, back_camera);
    else
      command = aimer.aim(targets, timestamp, 25, false);

    /// 发射逻辑
    // command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    if(command.control)
      gimbal->send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

  }
  rclcpp::shutdown();
  return 0;
}