#include "mt_detector.hpp"
#include <yaml-cpp/yaml.h>

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: yolo_(config_path, debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  auto model_path = yaml[yolo_name + "_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();

  auto model = core_.read_model(model_path);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();
  input.tensor().set_element_type(ov::element::u8).set_shape({1, 640, 640, 3}).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
  input.model().set_layout("NCHW");
  input.preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale(255.0);
  model = ppp.build();
  compiled_model_ = core_.compile_model(model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));
  tools::logger()->info("[MultiThreadDetector] initialized !");
}

void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  // 如果队列快满，丢最旧帧保证实时性
  if (queue_.size() >= 10) {
      std::tuple<cv::Mat, std::chrono::steady_clock::time_point, ov::InferRequest> dummy;
      if (queue_.try_pop(dummy)) {
        // 可选：统计丢帧数量（需要你在类里加一个计数器）
        // dropped_++;
      }
  }

  // 计算letterbox缩放
  auto scale = std::min(640.0 / img.rows, 640.0 / img.cols);
  auto h = static_cast<int>(img.rows * scale);
  auto w = static_cast<int>(img.cols * scale);

  // 复用缓冲，避免每帧分配大矩阵（thread_local 每个线程一份）
  thread_local cv::Mat input(640, 640, CV_8UC3);

  input.setTo(cv::Scalar(0, 0, 0));
  cv::resize(img, input(cv::Rect(0, 0, w, h)), {w, h});

  // ✅ 关键修复：让 OpenVINO 自己持有输入内存
  ov::Tensor in_tensor(ov::element::u8, {1, 640, 640, 3});
  std::memcpy(in_tensor.data(), input.data, 640 * 640 * 3);

  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(in_tensor);
  infer_request.start_async();

  // 这里不要 clone，先用浅拷贝（Mat 引用计数）
  // 如果你确认相机 read 会复用同一块buffer导致数据被覆盖，再改回 clone。
  queue_.push({img, t, std::move(infer_request)});
}


std::vector<std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>> 
MultiThreadDetector::get_all_results()
{
    std::vector<std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>> results;
    std::tuple<cv::Mat, std::chrono::steady_clock::time_point, ov::InferRequest> raw_data;
    std::tuple<cv::Mat, std::chrono::steady_clock::time_point, ov::InferRequest> latest_raw;
    bool has_data = false;

    // --- 优化点：只弹出，不等待，不处理 ---
    while (queue_.try_pop(raw_data)) {
        latest_raw = std::move(raw_data);
        has_data = true;
    }

    if (has_data) {
        // --- 仅对最后（最新）的一帧执行耗时操作 ---
        auto& [img, t, infer_request] = latest_raw;
        
        // 1. 等待这一帧推理完成 (只等一次)
        infer_request.wait(); 

        // 2. 仅对这一帧执行后处理 (NMS等耗时算法只算一次)
        auto output_tensor = infer_request.get_output_tensor();
        cv::Mat output(output_tensor.get_shape()[1], output_tensor.get_shape()[2], CV_32F, output_tensor.data());
        auto scale = std::min(640.0 / img.rows, 640.0 / img.cols);
        auto armors = yolo_.postprocess(scale, output, img, 0);

        results.push_back({std::move(img), std::move(armors), t});
    }

    return results;
}

// 保留原有的 pop 接口兼容旧代码
std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop() {
    auto [img, armors, t] = debug_pop();
    return {std::move(armors), t};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::debug_pop() {
    auto [img, t, infer_request] = queue_.pop();
    infer_request.wait();
    auto output_tensor = infer_request.get_output_tensor();
    cv::Mat output(output_tensor.get_shape()[1], output_tensor.get_shape()[2], CV_32F, output_tensor.data());
    auto scale = std::min(640.0 / img.rows, 640.0 / img.cols);
    auto armors = yolo_.postprocess(scale, output, img, 0);
    return {img, std::move(armors), t};
}

size_t MultiThreadDetector::backlog()  {
    return queue_.size();
}

} // namespace multithread
} // namespace auto_aim