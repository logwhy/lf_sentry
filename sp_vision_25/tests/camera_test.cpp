#include "io/camera.hpp"

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  io::Camera camera(config_path);

  // 录制器指针，初始为空
  std::unique_ptr<tools::Recorder> recorder;

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();

  // 提示用户操作方式
  if (display) {
    tools::logger()->info("Press 'r' in the window to START/STOP recording.");
    tools::logger()->info("Press 'q' to QUIT.");
  }

  while (!exiter.exit()) {
    camera.read(img, timestamp);

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    tools::logger()->info("{:.2f} fps", 1 / dt);

    // 如果录制器存在（正在录制），则写入当前帧
    if (recorder) {
      // camera_test 没有IMU数据，传入单位四元数占位
      recorder->record(img, Eigen::Quaterniond::Identity(), timestamp);
      // 在图像上绘制红点提示正在录制
      cv::circle(img, {30, 30}, 10, {0, 0, 255}, -1); 
    }

    if (!display) continue;
    cv::imshow("img", img);

    // 键盘事件处理
    int key = cv::waitKey(1);
    if (key == 'q') {
      break;
    } else if (key == 'r') {
      if (recorder) {
        // 如果正在录制，则停止（销毁对象会自动保存文件）
        recorder.reset();
        tools::logger()->info("Recording STOPPED. File saved.");
      } else {
        // 如果未录制，则开始（创建新对象）
        recorder = std::make_unique<tools::Recorder>(30.0);
        tools::logger()->info("Recording STARTED.");
      }
    }
  }
}