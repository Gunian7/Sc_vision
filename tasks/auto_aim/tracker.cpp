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
  jump_z_threshold_ = 0.02;
  jump_confirm_count_ = 2;
  jump_avg_alpha_ = 1.0;
  jump_fire_cooldown_ = 0.0;
  jump_fire_cooldown_min_ = 0.0;
  jump_fire_cooldown_max_ = 0.0;
  jump_fire_cooldown_speed_start_ = 0.0;
  jump_fire_cooldown_speed_end_ = 0.0;
  jump_fire_cooldown_dynamic_ = false;
  outpost_jump_fire_cooldown_ = 0.0;
  jump_min_interval_ = 0.0;
  process_noise_linear_normal_ = 100.0;
  process_noise_angular_normal_ = 400.0;
  process_noise_linear_outpost_ = 10.0;
  process_noise_angular_outpost_ = 0.1;
  measurement_noise_yaw_ = 2e-3;
  measurement_noise_pitch_ = 2e-3;
  if (yaml["jump_z_threshold"].IsDefined()) {
    jump_z_threshold_ = yaml["jump_z_threshold"].as<double>();
  }
  if (yaml["jump_confirm_count"].IsDefined()) {
    jump_confirm_count_ = yaml["jump_confirm_count"].as<int>();
  }
  if (yaml["jump_avg_alpha"].IsDefined()) {
    jump_avg_alpha_ = yaml["jump_avg_alpha"].as<double>();
  }
  if (yaml["jump_fire_cooldown"].IsDefined()) {
    jump_fire_cooldown_ = yaml["jump_fire_cooldown"].as<double>();
  }
  if (yaml["jump_fire_cooldown_min"].IsDefined()) {
    jump_fire_cooldown_min_ = yaml["jump_fire_cooldown_min"].as<double>();
    jump_fire_cooldown_dynamic_ = true;
  }
  if (yaml["jump_fire_cooldown_max"].IsDefined()) {
    jump_fire_cooldown_max_ = yaml["jump_fire_cooldown_max"].as<double>();
    jump_fire_cooldown_dynamic_ = true;
  }
  if (yaml["jump_fire_cooldown_speed_start"].IsDefined()) {
    jump_fire_cooldown_speed_start_ = yaml["jump_fire_cooldown_speed_start"].as<double>();
    jump_fire_cooldown_dynamic_ = true;
  }
  if (yaml["jump_fire_cooldown_speed_end"].IsDefined()) {
    jump_fire_cooldown_speed_end_ = yaml["jump_fire_cooldown_speed_end"].as<double>();
    jump_fire_cooldown_dynamic_ = true;
  }
  if (yaml["outpost_jump_fire_cooldown"].IsDefined()) {
    outpost_jump_fire_cooldown_ = yaml["outpost_jump_fire_cooldown"].as<double>();
  }
  if (yaml["jump_min_interval"].IsDefined()) {
    jump_min_interval_ = yaml["jump_min_interval"].as<double>();
  }
  if (yaml["target_process_noise_linear_normal"].IsDefined()) {
    process_noise_linear_normal_ = yaml["target_process_noise_linear_normal"].as<double>();
  }
  if (yaml["target_process_noise_angular_normal"].IsDefined()) {
    process_noise_angular_normal_ = yaml["target_process_noise_angular_normal"].as<double>();
  }
  if (yaml["target_process_noise_linear_outpost"].IsDefined()) {
    process_noise_linear_outpost_ = yaml["target_process_noise_linear_outpost"].as<double>();
  }
  if (yaml["target_process_noise_angular_outpost"].IsDefined()) {
    process_noise_angular_outpost_ = yaml["target_process_noise_angular_outpost"].as<double>();
  }
  if (yaml["target_measurement_noise_yaw"].IsDefined()) {
    measurement_noise_yaw_ = yaml["target_measurement_noise_yaw"].as<double>();
  }
  if (yaml["target_measurement_noise_pitch"].IsDefined()) {
    measurement_noise_pitch_ = yaml["target_measurement_noise_pitch"].as<double>();
  }
  force_target_angular_velocity_ = false;
  forced_target_angular_velocity_ = 0.0;
  if (yaml["force_target_angular_velocity"].IsDefined()) {
    force_target_angular_velocity_ = yaml["force_target_angular_velocity"].as<bool>();
  }
  if (yaml["forced_target_angular_velocity"].IsDefined()) {
    forced_target_angular_velocity_ = yaml["forced_target_angular_velocity"].as<double>();
  }

  if (force_target_angular_velocity_) {
    tools::logger()->warn(
      "[Tracker] force_target_angular_velocity=true, use fixed w={:.3f} rad/s",
      forced_target_angular_velocity_);
  }
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

// 处理全向感知和主相机的联合跟踪以及目标切换
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

// 更新状态机
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

// 设置新目标
bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  solver_.solve(armor);

  // 根据兵种优化初始化参数
  // 这个逻辑过时了记得改
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig(11);
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig(11);
    P0_dig << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 1e-4;
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig(11);
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig(11);
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  target_.set_jump_params(jump_z_threshold_, jump_confirm_count_);
  target_.set_jump_avg_alpha(jump_avg_alpha_);
  target_.set_process_noise(
    process_noise_linear_normal_, process_noise_angular_normal_, process_noise_linear_outpost_,
    process_noise_angular_outpost_);
  target_.set_measurement_noise(measurement_noise_yaw_, measurement_noise_pitch_);
  if (armor.name == ArmorName::outpost && outpost_jump_fire_cooldown_ > 0.0) {
    target_.set_jump_fire_cooldown(outpost_jump_fire_cooldown_);
  } else if (jump_fire_cooldown_dynamic_) {
    target_.set_jump_fire_cooldown_params(
      jump_fire_cooldown_min_, jump_fire_cooldown_max_, jump_fire_cooldown_speed_start_,
      jump_fire_cooldown_speed_end_);
  } else {
    target_.set_jump_fire_cooldown(jump_fire_cooldown_);
  }
  target_.set_jump_min_interval(jump_min_interval_);
  if (force_target_angular_velocity_) {
    target_.set_angular_velocity(forced_target_angular_velocity_);
  }

  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);
  if (force_target_angular_velocity_) {
    target_.set_angular_velocity(forced_target_angular_velocity_);
  }

  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;

    solver_.solve(armor);
    target_.update(armor);
    return true;
  }

  return false;
}

}  // namespace auto_aim