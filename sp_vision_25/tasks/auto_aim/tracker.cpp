#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  solver_.solve(armor);

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  // 1. 收集同类候选装甲板
  std::vector<std::reference_wrapper<Armor>> candidates;
  for (auto & armor : armors) {
    if (armor.name == target_.name && armor.type == target_.armor_type) {
      candidates.push_back(armor);
    }
  }

  if (candidates.empty()) return false;

  // 2. 预测 target 当前所有可能装甲板位置
  const auto pred_xyza_list = target_.armor_xyza_list();

  constexpr double SWITCH_MARGIN = 0.25;   // 只有明显更优才允许切换
  constexpr double MAX_ACCEPT_COST = 1.20; // 过大的匹配直接拒绝更新

  double best_cost = 1e9;
  Armor * best_armor = nullptr;

  // 用来判断“是否值得切换”
  double same_id_best_cost = 1e9;
  Armor * same_id_best_armor = nullptr;

  double other_best_cost = 1e9;
  Armor * other_best_armor = nullptr;

  const double angular_speed = std::abs(target_.ekf_x()[7]);

  // 3. 对每个候选观测进行打分
  for (auto & armor_ref : candidates) {
    auto & armor = armor_ref.get();

    solver_.solve(armor);

    // 观测的 ypd / yaw
    const auto & obs_ypd = armor.ypd_in_world;
    const auto & obs_yaw = armor.ypr_in_world[0];

    double min_cost_this_armor = 1e9;
    int best_pred_id_this_armor = -1;

    // 与预测的每一块装甲板比较，选这个观测最可能对应的那块
    for (int i = 0; i < static_cast<int>(pred_xyza_list.size()); ++i) {
      const auto & pred_xyza = pred_xyza_list[i];
      Eigen::Vector3d pred_xyz = pred_xyza.head<3>();
      Eigen::Vector3d pred_ypd = tools::xyz2ypd(pred_xyz);
      double pred_yaw = pred_xyza[3];

      // 代价函数：距离 + yaw差 + 视角差
      double cost_dist = std::abs(obs_ypd[2] - pred_ypd[2]);
      double cost_yaw = std::abs(tools::limit_rad(obs_yaw - pred_yaw));
      double cost_view = std::abs(tools::limit_rad(obs_ypd[0] - pred_ypd[0]));

      // 动态切换惩罚：低速时锁紧，高速时放松
      double switch_penalty = 0.0;
      if (i != target_.last_id) {
        if (angular_speed < 0.8) {
          switch_penalty = 0.8;
        } else if (angular_speed < 1.5) {
          switch_penalty = 0.35;
        } else {
          switch_penalty = 0.12;
        }
      }

      double cost = 2.0 * cost_dist + 1.5 * cost_yaw + 1.0 * cost_view + switch_penalty;

      // 每个 pred_id 的 cost
      static int tracker_cost_log_cnt = 0;
      if (tracker_cost_log_cnt++ % 20 == 0) {
        tools::logger()->info(
          "[TrackerCost] cand={} pred_id={} last_id={} dist={:.3f} yaw={:.3f} view={:.3f} sw={:.3f} total={:.3f}",
          static_cast<int>(armor.name),
          i,
          target_.last_id,
          cost_dist,
          cost_yaw,
          cost_view,
          switch_penalty,
          cost);
      }

      if (cost < min_cost_this_armor) {
        min_cost_this_armor = cost;
        best_pred_id_this_armor = i;
      }
    }

    // 这个候选 armor 在所有 pred_id 中的最佳匹配结果
    static int tracker_min_log_cnt = 0;
    if (tracker_min_log_cnt++ % 20 == 0) {
      tools::logger()->info(
        "[TrackerCost] cand={} best_pred_id={} min_cost={:.3f}",
        static_cast<int>(armor.name),
        best_pred_id_this_armor,
        min_cost_this_armor);
    }

    if (min_cost_this_armor < best_cost) {
      best_cost = min_cost_this_armor;
      best_armor = &armor;
    }

    // 统计“沿用 last_id”和“切到其他 id”的最佳候选
    if (best_pred_id_this_armor == target_.last_id) {
      if (min_cost_this_armor < same_id_best_cost) {
        same_id_best_cost = min_cost_this_armor;
        same_id_best_armor = &armor;
      }
    } else {
      if (min_cost_this_armor < other_best_cost) {
        other_best_cost = min_cost_this_armor;
        other_best_armor = &armor;
      }
    }
  }

  if (best_armor == nullptr) return false;

  // 4. 选择最终用于更新的候选
  Armor * selected_armor = best_armor;

  // 如果 same / other 都存在，则只有 other 明显更优才切换
  if (same_id_best_armor != nullptr && other_best_armor != nullptr) {
    if (other_best_cost + SWITCH_MARGIN < same_id_best_cost) {
      selected_armor = other_best_armor;
      best_cost = other_best_cost;
    } else {
      selected_armor = same_id_best_armor;
      best_cost = same_id_best_cost;
    }
  } else if (same_id_best_armor != nullptr) {
    selected_armor = same_id_best_armor;
    best_cost = same_id_best_cost;
  } else if (other_best_armor != nullptr) {
    selected_armor = other_best_armor;
    best_cost = other_best_cost;
  }

  static int tracker_best_log_cnt = 0;
  if (tracker_best_log_cnt++ % 10 == 0) {
    tools::logger()->warn(
      "[TrackerDecision] last_id={} same_cost={:.3f} other_cost={:.3f} diff(other-same)={:.3f} selected_center=({:.1f},{:.1f}) final_best={:.3f}",
      target_.last_id,
      same_id_best_cost,
      other_best_cost,
      other_best_cost - same_id_best_cost,
      selected_armor->center.x,
      selected_armor->center.y,
      best_cost);
  }

  // 5. 如果最佳匹配代价太大，宁可本帧不更新，也不要把 target 带崩
  if (best_cost > MAX_ACCEPT_COST) {
    static int tracker_reject_log_cnt = 0;
    if (tracker_reject_log_cnt++ % 10 == 0) {
      tools::logger()->warn(
        "[TrackerDecision] reject update, best_cost={:.3f} > {:.3f}",
        best_cost,
        MAX_ACCEPT_COST);
    }
    return false;
  }

  // 6. 一帧只用一个观测更新 target
  target_.update(*selected_armor);

  return true;
}
}  // namespace auto_aim