#include "yolo11_buff.hpp"

const double ConfidenceThreshold = 0.7f;
const double IouThreshold = 0.4f;
namespace auto_buff
{

// 将 OpenVINO 输出 Tensor 统一转换为 [C, N] 的 cv::Mat，便于后续解析。
// 支持常见两种形状：
//   1) [1, C, N]
//   2) [1, N, C]
static bool BuildDetOutputMat(const ov::Tensor & output, cv::Mat & det_output)
{
  const ov::Shape shape = output.get_shape();
  const float * output_buffer = output.data<const float>();

  if (shape.size() != 3) {
    return false;
  }

  const int d1 = static_cast<int>(shape[1]);
  const int d2 = static_cast<int>(shape[2]);

  // 经验：通道数(C)一般远小于候选框数量(N≈8400)
  if (d1 <= 256 && d2 > 256) {
    det_output = cv::Mat(d1, d2, CV_32F, (void *)output_buffer);  // [C, N]
    return true;
  }
  if (d2 <= 256 && d1 > 256) {
    // [N, C] -> [C, N]
    det_output = cv::Mat(d1, d2, CV_32F, (void *)output_buffer).t();
    return true;
  }

  // 兜底：按 [C, N] 处理
  det_output = cv::Mat(d1, d2, CV_32F, (void *)output_buffer);
  return true;
}

YOLO11_BUFF::YOLO11_BUFF(const std::string & config)
{
  auto yaml = YAML::LoadFile(config);
  std::string model_path = yaml["model"].as<std::string>();
  model = core.read_model(model_path);
  // printInputAndOutputsInfo(*model);  // 打印模型信息
  /// 载入并编译模型
  compiled_model = core.compile_model(model, "CPU");
  /// 创建推理请求
  infer_request = compiled_model.create_infer_request();
  // 获取模型输入节点
  input_tensor = infer_request.get_input_tensor();
  input_tensor.set_shape({1, 3, 640, 640});
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::get_multicandidateboxes(cv::Mat & image)
{
  const int64 start = cv::getTickCount();

  if (image.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::vector<YOLO11_BUFF::Object>();
  }

  // 统一使用与 get_onecandidatebox 相同的预处理（CHW float / 归一化 / BGR->RGB）
  const float factor = fill_tensor_data_image(input_tensor, image);

  // 推理
  infer_request.infer();

  // 读取输出
  const ov::Tensor output = infer_request.get_output_tensor();
  cv::Mat det_output;
  if (!BuildDetOutputMat(output, det_output)) {
    tools::logger()->error("Unexpected model output shape, cannot parse.");
    return std::vector<YOLO11_BUFF::Object>();
  }

  const int out_rows = det_output.rows;
  const int out_cols = det_output.cols;

  // ---------- 自动推断关键点布局 ----------
  // 输出通道一般为：
  //   4(box) + 1(obj_conf) + [nc(class)] + 6*(2/3)(kpts)
  int kpt_start = -1;
  int kpt_dim = -1;

  for (int s = 5; s <= std::min(out_rows - 1, 5 + 64); ++s) {
    const int rem = out_rows - s;
    if (rem <= 0) continue;

    if (rem % (NUM_POINTS * 3) == 0) {
      kpt_start = s;
      kpt_dim = 3;  // (x,y,v)
      break;
    }
    if (rem % (NUM_POINTS * 2) == 0) {
      kpt_start = s;
      kpt_dim = 2;  // (x,y)
      break;
    }
  }

  if (kpt_start < 0 || kpt_dim < 0) {
    tools::logger()->error("Output channels not match NUM_POINTS.");
    return std::vector<YOLO11_BUFF::Object>();
  }

  const int cls_start = 5;
  const int cls_end = kpt_start;  // [5, kpt_start) 若存在则认为是类别分数

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> labels;
  std::vector<std::vector<float>> objects_keypoints;

  boxes.reserve(out_cols);
  confidences.reserve(out_cols);
  labels.reserve(out_cols);
  objects_keypoints.reserve(out_cols);

  for (int i = 0; i < out_cols; ++i) {
    const float obj_conf = det_output.at<float>(4, i);
    if (obj_conf <= 0.f) continue;

    // 如果存在类别通道：score = obj_conf * max_cls_conf
    float cls_conf = 1.f;
    int cls_id = 0;
    if (cls_end > cls_start) {
      cls_conf = det_output.at<float>(cls_start, i);
      cls_id = 0;
      for (int c = cls_start + 1; c < cls_end; ++c) {
        const float v = det_output.at<float>(c, i);
        if (v > cls_conf) {
          cls_conf = v;
          cls_id = c - cls_start;
        }
      }
    }

    const float score = obj_conf * cls_conf;
    if (score < ConfidenceThreshold) continue;

    // bbox
    const float cx = det_output.at<float>(0, i);
    const float cy = det_output.at<float>(1, i);
    const float ow = det_output.at<float>(2, i);
    const float oh = det_output.at<float>(3, i);

    cv::Rect box;
    box.x = static_cast<int>((cx - 0.5f * ow) * factor);
    box.y = static_cast<int>((cy - 0.5f * oh) * factor);
    box.width = static_cast<int>(ow * factor);
    box.height = static_cast<int>(oh * factor);

    boxes.push_back(box);
    confidences.push_back(score);
    labels.push_back(cls_id);

    // keypoints
    std::vector<float> keypoints;
    keypoints.reserve(NUM_POINTS * 2);

    cv::Mat kpts = det_output.col(i).rowRange(kpt_start, kpt_start + NUM_POINTS * kpt_dim);
    for (int j = 0; j < NUM_POINTS; ++j) {
      const float x = kpts.at<float>(j * kpt_dim + 0, 0) * factor;
      const float y = kpts.at<float>(j * kpt_dim + 1, 0) * factor;
      keypoints.push_back(x);
      keypoints.push_back(y);
    }
    objects_keypoints.push_back(std::move(keypoints));
  }

  // NMS
  std::vector<int> indexes;
  cv::dnn::NMSBoxes(boxes, confidences, ConfidenceThreshold, IouThreshold, indexes);

  std::vector<Object> object_result;
  object_result.reserve(indexes.size());

  for (size_t n = 0; n < indexes.size(); ++n) {
    Object obj;
    const int index = indexes[n];

    obj.rect = boxes[index];
    obj.prob = confidences[index];
    obj.label = labels[index];

    const std::vector<float> & keypoint = objects_keypoints[index];
    if ((int)keypoint.size() < NUM_POINTS * 2) continue;

    obj.kpt.reserve(NUM_POINTS);
    for (int j = 0; j < NUM_POINTS; ++j) {
      const float x_coord = keypoint[j * 2];
      const float y_coord = keypoint[j * 2 + 1];
      obj.kpt.emplace_back(x_coord, y_coord);
    }

    object_result.push_back(obj);

    // 绘制关键点
    const int radius = 2;
    for (int j = 0; j < NUM_POINTS; ++j) {
      cv::circle(image, obj.kpt[j], radius, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
    }
  }

  // FPS
  const float t = (cv::getTickCount() - start) / static_cast<float>(cv::getTickFrequency());
  cv::putText(
    image, cv::format("FPS: %.2f", 1.0 / t), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0,
    cv::Scalar(255, 0, 0), 2, 8);

  return object_result;
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::get_onecandidatebox(cv::Mat & image)
{
  const int64 start = cv::getTickCount();

  if (image.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::vector<YOLO11_BUFF::Object>();
  }

  // 预处理
  const float factor = fill_tensor_data_image(input_tensor, image);

  // 推理
  infer_request.infer();

  // 输出
  const ov::Tensor output = infer_request.get_output_tensor();
  cv::Mat det_output;
  if (!BuildDetOutputMat(output, det_output)) {
    tools::logger()->error("Unexpected model output shape, cannot parse.");
    return std::vector<YOLO11_BUFF::Object>();
  }

  const int out_rows = det_output.rows;
  const int out_cols = det_output.cols;

  // ---------- 自动推断关键点布局 ----------
  int kpt_start = -1;
  int kpt_dim = -1;

  for (int s = 5; s <= std::min(out_rows - 1, 5 + 64); ++s) {
    const int rem = out_rows - s;
    if (rem <= 0) continue;

    if (rem % (NUM_POINTS * 3) == 0) {
      kpt_start = s;
      kpt_dim = 3;
      break;
    }
    if (rem % (NUM_POINTS * 2) == 0) {
      kpt_start = s;
      kpt_dim = 2;
      break;
    }
  }

  if (kpt_start < 0 || kpt_dim < 0) {
    tools::logger()->error("Output channels not match NUM_POINTS.");
    return std::vector<YOLO11_BUFF::Object>();
  }

  const int cls_start = 5;
  const int cls_end = kpt_start;

  // ---------- 找到最大 score 的框 ----------
  int best_index = -1;
  int best_label = 0;
  float best_score = 0.0f;

  for (int i = 0; i < out_cols; ++i) {
    const float obj_conf = det_output.at<float>(4, i);
    if (obj_conf <= 0.f) continue;

    float cls_conf = 1.f;
    int cls_id = 0;
    if (cls_end > cls_start) {
      cls_conf = det_output.at<float>(cls_start, i);
      cls_id = 0;
      for (int c = cls_start + 1; c < cls_end; ++c) {
        const float v = det_output.at<float>(c, i);
        if (v > cls_conf) {
          cls_conf = v;
          cls_id = c - cls_start;
        }
      }
    }

    const float score = obj_conf * cls_conf;
    if (score > best_score) {
      best_score = score;
      best_index = i;
      best_label = cls_id;
    }
  }

  std::vector<Object> object_result;

  if (best_index >= 0 && best_score > ConfidenceThreshold) {
    Object obj;

    // bbox
    const float cx = det_output.at<float>(0, best_index);
    const float cy = det_output.at<float>(1, best_index);
    const float ow = det_output.at<float>(2, best_index);
    const float oh = det_output.at<float>(3, best_index);

    obj.rect.x = static_cast<int>((cx - 0.5f * ow) * factor);
    obj.rect.y = static_cast<int>((cy - 0.5f * oh) * factor);
    obj.rect.width = static_cast<int>(ow * factor);
    obj.rect.height = static_cast<int>(oh * factor);

    obj.prob = best_score;
    obj.label = best_label;

    // keypoints
    cv::Mat kpts = det_output.col(best_index).rowRange(kpt_start, kpt_start + NUM_POINTS * kpt_dim);
    obj.kpt.reserve(NUM_POINTS);

    std::vector<cv::Point2f> kpt_temp;

    for (int i = 0; i < NUM_POINTS; ++i) {
      const float x = kpts.at<float>(i * kpt_dim + 0, 0) * factor;
      const float y = kpts.at<float>(i * kpt_dim + 1, 0) * factor;
      kpt_temp.emplace_back(x, y);
    }

    // auto temp= obj.kpt[0];
    // obj.kpt[0] = obj.kpt[2];
    // obj.kpt[2] = temp;
    // temp = obj.kpt[1];
    // obj.kpt[1] = obj.kpt[3];
    // obj.kpt[3] = temp;    
    obj.kpt.emplace_back(kpt_temp[4]);
    obj.kpt.emplace_back(kpt_temp[6]);
    obj.kpt.emplace_back(kpt_temp[0]);
    obj.kpt.emplace_back(kpt_temp[2]);
    obj.kpt.emplace_back(kpt_temp[8]);
    obj.kpt.emplace_back(kpt_temp[9]);

    object_result.push_back(obj);

    // 绘制
    cv::rectangle(image, obj.rect, cv::Scalar(255, 255, 255), 1, 8);

    std::string cls_name = "buff";
    if (!class_names.empty() && obj.label >= 0 && obj.label < (int)class_names.size()) {
      cls_name = class_names[obj.label];
    }
    const std::string label = cls_name + ":" + std::to_string(obj.prob).substr(0, 4);

    const cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
    const cv::Rect textBox(
      obj.rect.tl().x, obj.rect.tl().y - 15, textSize.width, textSize.height + 5);
    cv::rectangle(image, textBox, cv::Scalar(0, 255, 255), cv::FILLED);
    cv::putText(
      image, label, cv::Point(obj.rect.tl().x, obj.rect.tl().y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(0, 0, 0));

    const int radius = 2;
    for (int i = 0; i < (int)obj.kpt.size(); ++i) {
      cv::circle(image, obj.kpt[i], radius, cv::Scalar(255, 255, 0), -1, cv::LINE_AA);
      cv::putText(
        image, std::to_string(i + 1), obj.kpt[i] + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX,
        0.5, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
    }
  }

  // FPS
  const float t = (cv::getTickCount() - start) / static_cast<float>(cv::getTickFrequency());
  cv::putText(
    image, cv::format("FPS: %.2f", 1.0 / t), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0,
    cv::Scalar(255, 0, 0), 2, 8);

  return object_result;
}

void YOLO11_BUFF::convert(
  const cv::Mat & input, cv::Mat & output, const bool normalize, const bool BGR2RGB) const
{
  input.convertTo(output, CV_32F);
  if (normalize) output = output / 255.0;  // 归一化到[0, 1]
  if (BGR2RGB) cv::cvtColor(output, output, cv::COLOR_BGR2RGB);
}

float YOLO11_BUFF::fill_tensor_data_image(ov::Tensor & input_tensor, const cv::Mat & input_image) const
{
  /// letterbox变换: 不改变宽高比(aspect ratio), 将input_image缩放并放置到blob_image左上角
  const ov::Shape tensor_shape = input_tensor.get_shape();
  const size_t num_channels = tensor_shape[1];
  const size_t height = tensor_shape[2];
  const size_t width = tensor_shape[3];
  // 缩放因子
  const float scale = std::min(height / float(input_image.rows), width / float(input_image.cols));
  const cv::Matx23f matrix{
    scale, 0.0, 0.0, 0.0, scale, 0.0,
  };
  cv::Mat blob_image;
  // 下面根据scale范围进行数据转换, 这只是为了提高一点速度(主要是提高了交换通道的速度)
  // 如果不在意这点速度提升的可以固定一种做法(两个if分支随便一个都可以)
  if (scale < 1.0f) {
    // 要缩小, 那么先缩小再交换通道
    cv::warpAffine(input_image, blob_image, matrix, cv::Size(width, height));
    convert(blob_image, blob_image, true, true);
  } else {
    // 要放大, 那么先交换通道再放大
    convert(input_image, blob_image, true, true);
    cv::warpAffine(blob_image, blob_image, matrix, cv::Size(width, height));
  }

  /// 将图像数据填入input_tensor
  float * const input_tensor_data = input_tensor.data<float>();
  // 原有图片数据为 HWC格式，模型输入节点要求的为 CHW 格式
  for (size_t c = 0; c < num_channels; c++) {
    for (size_t h = 0; h < height; h++) {
      for (size_t w = 0; w < width; w++) {
        input_tensor_data[c * width * height + h * width + w] =
          blob_image.at<cv::Vec<float, 3>>(h, w)[c];
      }
    }
  }
  return 1 / scale;
}

void YOLO11_BUFF::printInputAndOutputsInfo(const ov::Model & network)
{
  std::cout << "model name: " << network.get_friendly_name() << std::endl;

  const std::vector<ov::Output<const ov::Node>> inputs = network.inputs();
  for (const ov::Output<const ov::Node> & input : inputs) {
    std::cout << "    inputs" << std::endl;

    const std::string name = input.get_names().empty() ? "NONE" : input.get_any_name();
    std::cout << "        input name: " << name << std::endl;

    const ov::element::Type type = input.get_element_type();
    std::cout << "        input type: " << type << std::endl;

    const ov::Shape shape = input.get_shape();
    std::cout << "        input shape: " << shape << std::endl;
  }

  const std::vector<ov::Output<const ov::Node>> outputs = network.outputs();
  for (const ov::Output<const ov::Node> & output : outputs) {
    std::cout << "    outputs" << std::endl;

    const std::string name = output.get_names().empty() ? "NONE" : output.get_any_name();
    std::cout << "        output name: " << name << std::endl;

    const ov::element::Type type = output.get_element_type();
    std::cout << "        output type: " << type << std::endl;

    const ov::Shape shape = output.get_shape();
    std::cout << "        output shape: " << shape << std::endl;
  }
}

void YOLO11_BUFF::save(const std::string & programName, const cv::Mat & image)
{
  const std::filesystem::path saveDir = "../result/";
  if (!std::filesystem::exists(saveDir)) {
    std::filesystem::create_directories(saveDir);
  }
  const std::filesystem::path savePath = saveDir / (programName + ".jpg");
  cv::imwrite(savePath.string(), image);
}
}  // namespace auto_buff
