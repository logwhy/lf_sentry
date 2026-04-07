#include "decider.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace omniperception
{
Decider::Decider(const std::string & config_path) : detector_(config_path), count_(0)
{
  auto yaml = YAML::LoadFile(config_path);
  img_width_ = yaml["image_width"].as<double>();
  img_height_ = yaml["image_height"].as<double>();
  fov_h_ = yaml["fov_h"].as<double>();
  fov_v_ = yaml["fov_v"].as<double>();
  new_fov_h_ = yaml["new_fov_h"].as<double>();
  new_fov_v_ = yaml["new_fov_v"].as<double>();
  enemy_color_ =
    (yaml["enemy_color"].as<std::string>() == "red") ? auto_aim::Color::red : auto_aim::Color::blue;
  mode_ = yaml["mode"].as<double>();
}

io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::USBCamera & usbcam1,
  io::USBCamera & usbcam2, io::Camera & back_camera)
{
  Eigen::Vector2d delta_angle;
  io::USBCamera * cams[] = {&usbcam1, &usbcam2};

  cv::Mat usb_img;
  std::chrono::steady_clock::time_point timestamp;
  if (count_ < 0 || count_ > 2) {
    throw std::runtime_error("count_ out of valid range [0,2]");
  }
  if (count_ == 2) {
    back_camera.read(usb_img, timestamp);
  } else {
    cams[count_]->read(usb_img, timestamp);
  }
  auto armors = yolo.detect(usb_img);
  auto empty = armor_filter(armors);

  if (!empty) {
    if (count_ == 2) {
      delta_angle = this->delta_angle(armors, "back");
    } else {
      delta_angle = this->delta_angle(armors, cams[count_]->device_name);
    }

    // tools::logger()->debug(
    //   "[{} camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}",
    //   (count_ == 2 ? "back" : cams[count_]->device_name), delta_angle[0], delta_angle[1],
    //   armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    count_ = (count_ + 1) % 3;

    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  count_ = (count_ + 1) % 3;
  // 如果没有找到目标，返回默认命令
  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::Camera & back_camera)
{
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  back_camera.read(img, timestamp);
  auto armors = yolo.detect(img);
  auto empty = armor_filter(armors);

  if (!empty) {
    auto delta_angle = this->delta_angle(armors, "back");
    tools::logger()->debug(
      "[back camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}",
      delta_angle[0], delta_angle[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    // 1. 计算目标弧度 (gimbal_pos 为弧度，delta_angle 为角度，除以 57.3 转弧度)
    double target_yaw = gimbal_pos[0] + delta_angle[0] / 57.3;

    // 2. 显式归一化处理：确保角度在 [-PI, PI] 之间 (对应 [-180, 180] 度)
    // 57.3 约等于 180/PI，这里使用更精确的 PI 值进行回环判断
    const double PI = 3.14159265358979323846;
    while (target_yaw > PI) target_yaw -= 2 * PI;
    while (target_yaw < -PI) target_yaw += 2 * PI;

    return io::Command{
      true, 
      false, 
      target_yaw, // 使用归一化后的 yaw
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::SNCamera & back_camera)
{
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  
  // SNCamera 也有 read 接口，直接调用
  back_camera.read(img, timestamp);
  
  auto armors = yolo.detect(img);
  auto empty = armor_filter(armors);

  if (!empty) {
    auto delta_angle = this->delta_angle(armors, "back");

    // 1. 计算目标弧度
    double target_yaw = gimbal_pos[0] + delta_angle[0] / 57.3;

    // 2. 显式归一化处理
    const double PI = 3.14159265358979323846;
    while (target_yaw > PI) target_yaw -= 2 * PI;
    while (target_yaw < -PI) target_yaw += 2 * PI;

    if(delta_angle[0] > 180.0) {
      delta_angle[0] -= 360.0; // 将偏航角调整到 [-180, 180] 范围内
    }
    tools::logger()->debug(
      "[SNCamera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}",
      delta_angle[0], delta_angle[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    return io::Command{
      true, 
      false, 
      target_yaw, // 使用归一化后的 yaw
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(const std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) {
    return io::Command{false, false, 0, 0};
  }

  DetectionResult dr = detection_queue.front();
  if (dr.armors.empty()) return io::Command{false, false, 0, 0};
  tools::logger()->info(
    "omniperceptron find {},delta yaw is {:.4f}", auto_aim::ARMOR_NAMES[dr.armors.front().name],
    dr.delta_yaw * 57.3);

  return io::Command{true, false, dr.delta_yaw, dr.delta_pitch};
};
io::Command Decider::decide_by_armors(
  std::list<auto_aim::Armor> & armors, 
  const Eigen::Vector3d & gimbal_pos, 
  const std::string & camera_name)
{
  // 1. 过滤与优先级排序
  bool empty = armor_filter(armors);

  if (!empty) {
    // 2. 计算角度偏差
    auto delta = this->delta_angle(armors, camera_name);
    
    // 3. 计算目标 Yaw (弧度)
    double target_yaw = gimbal_pos[0] + delta[0] / 57.3;

    // 4. 角度归一化 (-PI ~ PI)
    const double PI = 3.14159265358979323846;
    while (target_yaw > PI) target_yaw -= 2 * PI;
    while (target_yaw < -PI) target_yaw += 2 * PI;

    // 5. 日志调试 (可选，为了性能可以注释掉)
    // tools::logger()->debug("[{}] delta yaw:{:.2f}", camera_name, delta[0]);

    return io::Command{
      true, 
      false, 
      target_yaw, 
      tools::limit_rad(delta[1] / 57.3)};
  }

  // 没找到目标
  return io::Command{false, false, 0, 0};
}

Eigen::Vector2d Decider::delta_angle(
  const std::list<auto_aim::Armor> & armors, const std::string & camera)
{
  Eigen::Vector2d delta_angle;

  if (camera == "left") {
    delta_angle[0] = 62 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }
  else if (camera == "right") {
    delta_angle[0] = -62 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }
  // ✅ 新增：dual 前向相机（不加方位偏置）
  else if (camera == "front" || camera == "FRONT") {
    delta_angle[0] = (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }
  // ✅ 新增：dual 后向相机（建议 180° 偏置；也可以替换成你的标定值）
  else if (camera == "back" || camera == "BACK") {
    delta_angle[0] = 180 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }
  // ✅ 保留原有：旧的 omni/back USB 逻辑（不破坏原功能）
  else {
    delta_angle[0] = 170 + (54.2 / 2) - armors.front().center_norm.x * 54.2;
    delta_angle[1] = armors.front().center_norm.y * 44.5 - 44.5 / 2;
    return delta_angle;
  }
}


bool Decider::armor_filter(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return true;
  // 过滤非敌方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 25赛季没有5号装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::five; });
  // 不打工程
  // armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::two; });
  // 不打前哨站
  armors.remove_if(
    [&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::outpost; });

  // 过滤掉刚复活无敌的装甲板
  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(invincible_armor_.begin(), invincible_armor_.end(), a.name) !=
           invincible_armor_.end();
  });

  return armors.empty();
}

void Decider::set_priority(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return;

  const PriorityMap & priority_map = (mode_ == MODE_ONE) ? mode1 : mode2;

  if (!armors.empty()) {
    for (auto & armor : armors) {
      armor.priority = priority_map.at(armor.name);
    }
  }
}

void Decider::sort(std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) return;

  // 对每个 DetectionResult 调用 armor_filter 和 set_priority
  for (auto & dr : detection_queue) {
    armor_filter(dr.armors);
    set_priority(dr.armors);

    // 对每个 DetectionResult 中的 armors 进行排序
    dr.armors.sort(
      [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });
  }

  // 根据优先级对 DetectionResult 进行排序
  std::sort(
    detection_queue.begin(), detection_queue.end(),
    [](const DetectionResult & a, const DetectionResult & b) {
      return a.armors.front().priority < b.armors.front().priority;
    });
}

Eigen::Vector4d Decider::get_target_info(
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets)
{
  if (armors.empty() || targets.empty()) return Eigen::Vector4d::Zero();

  auto target = targets.front();

  for (const auto & armor : armors) {
    if (armor.name == target.name) {
      return Eigen::Vector4d{
        armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], 1,
        static_cast<double>(armor.name) + 1};  //避免歧义+1(详见通信协议)
    }
  }

  return Eigen::Vector4d::Zero();
}

void Decider::get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids)
{
  invincible_armor_.clear();

  if (invincible_enemy_ids.empty()) return;

  for (const auto & id : invincible_enemy_ids) {
    // tools::logger()->info("invincible armor id: {}", id);
    invincible_armor_.push_back(auto_aim::ArmorName(id - 1));
  }
}

void Decider::get_auto_aim_target(
  std::list<auto_aim::Armor> & armors, const std::vector<int8_t> & auto_aim_target)
{
  if (auto_aim_target.empty()) return;

  std::vector<auto_aim::ArmorName> auto_aim_targets;

  for (const auto & target : auto_aim_target) {
    if (target <= 0 || static_cast<size_t>(target) > auto_aim::ARMOR_NAMES.size()) {
      // tools::logger()->warn("Received invalid auto_aim target value: {}", int(target));
      continue;
    }
    auto_aim_targets.push_back(static_cast<auto_aim::ArmorName>(target - 1));
    // tools::logger()->info("nav send auto_aim target is {}", auto_aim::ARMOR_NAMES[target - 1]);
  }

  if (auto_aim_targets.empty()) return;

  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(auto_aim_targets.begin(), auto_aim_targets.end(), a.name) ==
           auto_aim_targets.end();
  });
}

}  // namespace omniperception