#ifndef SP_VISION_25_SRC_ASYNC_DETECT_WORKER_HPP_
#define SP_VISION_25_SRC_ASYNC_DETECT_WORKER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <fmt/core.h>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/debug_tool.hpp"

namespace sp_vision
{

using SteadyClock = std::chrono::steady_clock;

struct ScopedTimer
{
  double & out_ms;
  SteadyClock::time_point start;

  explicit ScopedTimer(double & target) : out_ms(target), start(SteadyClock::now()) {}

  ~ScopedTimer()
  {
    out_ms = std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
  }
};

struct DetectPacket
{
  bool valid = false;
  uint64_t seq = 0;
  SteadyClock::time_point ts{};
  std::list<auto_aim::Armor> armors;
  double detect_ms = 0.0;
  double frame_age_ms = 0.0;
};

class AsyncDetectWorker
{
public:
  AsyncDetectWorker(
    std::string name,
    const std::string & camera_config,
    tools::DebugTool * debug,
    int idx)
  : name_(std::move(name)),
    camera_config_(camera_config),
    camera_(std::make_unique<io::SNCamera>(camera_config)),
    debug_(debug),
    idx_(idx)
  {
    if (debug_) debug_->set_cam_name(idx_, name_);
  }

  ~AsyncDetectWorker()
  {
    stop();
  }

  AsyncDetectWorker(const AsyncDetectWorker &) = delete;
  AsyncDetectWorker & operator=(const AsyncDetectWorker &) = delete;

  void start(const std::string & yolo_config)
  {
    if (running_) return;

    yolo_config_ = yolo_config;
    yolo_ = std::make_unique<auto_aim::YOLO>(yolo_config_, false);

    running_ = true;
    thread_ = std::thread(&AsyncDetectWorker::run, this);
  }

  void stop()
  {
    running_ = false;
    if (thread_.joinable()) thread_.join();
  }

  bool latest(DetectPacket & out) const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!packet_.valid) return false;
    out = packet_;
    return true;
  }

  const std::string & name() const { return name_; }

private:
  void run()
  {
    uint64_t local_seq = 0;
    int frame_count = 0;

    while (running_) {
      try {
        cv::Mat img;
        SteadyClock::time_point ts;
        camera_->read(img, ts);

        if (img.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }

        const double age_ms =
          std::chrono::duration<double, std::milli>(SteadyClock::now() - ts).count();
        if (debug_) debug_->record_cam_frame(idx_, age_ms);

        double detect_ms = 0.0;
        std::list<auto_aim::Armor> armors;
        {
          ScopedTimer timer(detect_ms);
          armors = yolo_->detect(img, frame_count++);
        }
        if (debug_) debug_->add_yolo_detect_ms(detect_ms);

        {
          std::lock_guard<std::mutex> lk(mtx_);
          packet_.valid = true;
          packet_.seq = ++local_seq;
          packet_.ts = ts;
          packet_.armors = std::move(armors);
          packet_.detect_ms = detect_ms;
          packet_.frame_age_ms = age_ms;
        }
      } catch (const std::exception & e) {
        fmt::print("[WARN] AsyncDetectWorker[{}] failed: {}\n", name_, e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } catch (...) {
        fmt::print("[WARN] AsyncDetectWorker[{}] failed with unknown error.\n", name_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  std::string name_;
  std::string camera_config_;
  std::string yolo_config_;

  std::unique_ptr<io::SNCamera> camera_;
  std::unique_ptr<auto_aim::YOLO> yolo_;

  tools::DebugTool * debug_ = nullptr;
  int idx_ = 0;

  mutable std::mutex mtx_;
  DetectPacket packet_;

  std::thread thread_;
  std::atomic_bool running_{false};
};

}  // namespace sp_vision

#endif  // SP_VISION_25_SRC_ASYNC_DETECT_WORKER_HPP_
