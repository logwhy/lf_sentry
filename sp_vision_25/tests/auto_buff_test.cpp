#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/uav.yaml    | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@input-path    |                        | avi文件的路径(不需要txt)}";

// 默认帧率，用于模拟时间流逝
int fps = 30;
int wait_time = 1000 / fps;

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  // Modified: 只打开视频，不打开txt
  auto video_path = fmt::format("{}.avi", input_path);
  cv::VideoCapture video(video_path);
  
  if (!video.isOpened()) {
      SPDLOG_ERROR("无法打开视频文件: {}", video_path);
      return -1;
  }

  auto_buff::Buff_Detector detector(config_path);
  auto_buff::Solver solver(config_path);
  // auto_buff::SmallTarget target;
  auto_buff::BigTarget target;
  auto_buff::Aimer aimer(config_path);

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();

  io::Command last_command;

  // 跳转到指定帧
  video.set(cv::CAP_PROP_POS_FRAMES, start_index);

  // Modified: 移除跳过txt行的循环

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    // Modified: 模拟生成数据
    // 1. 模拟时间戳：假设每帧间隔 1/fps 秒
    double simulated_time_sec = (double)(frame_count - start_index) / fps;
    auto timestamp = t0 + std::chrono::microseconds(int(simulated_time_sec * 1e6));

    // 2. 模拟IMU四元数：假设云台静止且正向 [w, x, y, z] = [1, 0, 0, 0]
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
    
    // 设置虚拟的云台姿态
    solver.set_R_gimbal2world({w, x, y, z});

    /// 自瞄核心逻辑
    auto power_runes = detector.detect(img);

    solver.solve(power_runes);

    target.get_target(power_runes, timestamp);

    auto target_copy = target;
    auto command = aimer.aim(target_copy, timestamp, 22, false);

    // -------------- 调试输出 --------------

    nlohmann::json data;

    // buff原始观测数据
    if (power_runes.has_value()) {
      const auto & p = power_runes.value();
      data["buff_R_yaw"] = p.ypd_in_world[0];
      data["buff_R_pitch"] = p.ypd_in_world[1];
      data["buff_R_dis"] = p.ypd_in_world[2];
      data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
      data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
      data["buff_roll"] = p.ypr_in_world[2] * 57.3;
    }

    if (!target.is_unsolve()) {
      auto & p = power_runes.value();

      // 显示识别点
      for (int i = 0; i < 4; i++) tools::draw_point(img, p.target().points[i]);
      tools::draw_point(img, p.target().center, {0, 0, 255}, 3);
      tools::draw_point(img, p.r_center, {0, 0, 255}, 3);

      // --- 注意：以下重投影逻辑在相机移动时会不准确，因为使用了虚拟IMU数据 ---
      
      // 当前帧target更新后buff
      auto Rxyz_in_world_now = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      auto image_points =
        solver.reproject_buff(Rxyz_in_world_now, target.ekf_x()[4], target.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {0, 255, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {0, 255, 0});

      // buff瞄准位置(预测)
      double dangle = target.ekf_x()[5] - target_copy.ekf_x()[5];
      auto Rxyz_in_world_pre = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      image_points =
        solver.reproject_buff(Rxyz_in_world_pre, target_copy.ekf_x()[4], target_copy.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {255, 0, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {255, 0, 0});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["R_yaw"] = x[0];
      data["R_V_yaw"] = x[1];
      data["R_pitch"] = x[2];
      data["R_dis"] = x[3];
      data["yaw"] = x[4] * 57.3;

      data["angle"] = x[5] * 57.3;
      data["spd"] = x[6] * 57.3;
      if (x.size() >= 10) {
        data["spd"] = x[6];
        data["a"] = x[7];
        data["w"] = x[8];
        data["fi"] = x[9];
        data["spd0"] = target.spd;
      }
    }

    // 云台响应情况
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
    }

    plotter.plot(data);

    // 在左上角提示这是无TXT模式
    cv::putText(img, "NO TXT MODE (Fake IMU)", {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 0, 255}, 2);
    cv::imshow("result", img);

    int key = cv::waitKey(wait_time);
    if (key == 'q') break;
    while (key == ' ') {
      int y = cv::waitKey(30);
      if (y == 'q') break;
    }
  }
  cv::destroyAllWindows();
  // text.close(); // 已移除

  return 0;
}