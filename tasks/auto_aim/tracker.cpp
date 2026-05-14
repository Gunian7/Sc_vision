#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

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
  omni_target_priority_{ArmorPriority::fifth},
  imm_enabled_(true),
  motion_state_enabled_(true),
  motion_w_low_(1.2),
  motion_w_high_(6.0),
  motion_dw_high_(8.0),
  imm_initialized_(false),
  imm_last_w_(0.0),
  imm_dw_lpf_(0.0),
  imm_last_t_(std::chrono::steady_clock::now())
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
  jump_z_threshold_ = 0.02;
  jump_yaw_threshold_rad_ = 40.0 / 57.3;
  jump_confirm_count_ = 2;
  jump_avg_alpha_ = 1.0;
  jump_fire_cooldown_ = 0.0;
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
  if (yaml["jump_yaw_threshold_deg"].IsDefined()) {
    jump_yaw_threshold_rad_ = yaml["jump_yaw_threshold_deg"].as<double>() / 57.3;
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
  if (yaml["motion_state_enable"].IsDefined()) {
    motion_state_enabled_ = yaml["motion_state_enable"].as<bool>();
  }
  if (yaml["enable_imm"].IsDefined()) {
    imm_enabled_ = yaml["enable_imm"].as<bool>();
  }
  if (yaml["motion_w_low"].IsDefined()) {
    motion_w_low_ = yaml["motion_w_low"].as<double>();
  }
  if (yaml["motion_w_high"].IsDefined()) {
    motion_w_high_ = yaml["motion_w_high"].as<double>();
  }
  if (yaml["motion_dw_high"].IsDefined()) {
    motion_dw_high_ = yaml["motion_dw_high"].as<double>();
  }
  SpinIMM::Params imm_params;
  imm_params.transition << 0.93, 0.05, 0.02, 0.04, 0.93, 0.03, 0.03, 0.07, 0.90;
  imm_params.r_yaw = 2e-3;
  imm_params.q_slow << 1e-4, 8e-2, 1e-1;
  imm_params.q_constant << 1e-4, 2e-2, 5e-2;
  imm_params.q_variable << 2e-4, 2e-1, 8e-1;
  imm_params.alpha_decay_slow = 0.2;
  imm_params.alpha_decay_constant = 0.5;
  imm_params.dt_min = 1e-3;
  imm_params.dt_max = 0.2;

  if (yaml["imm_transition"].IsDefined()) {
    const auto values = yaml["imm_transition"].as<std::vector<double>>();
    if (values.size() == 9) {
      imm_params.transition << values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8];
    }
  }
  if (yaml["imm_r_yaw"].IsDefined()) {
    imm_params.r_yaw = std::max(1e-9, yaml["imm_r_yaw"].as<double>());
  }
  if (yaml["imm_q_slow"].IsDefined()) {
    const auto values = yaml["imm_q_slow"].as<std::vector<double>>();
    if (values.size() == 3) {
      imm_params.q_slow << values[0], values[1], values[2];
    }
  }
  if (yaml["imm_q_constant"].IsDefined()) {
    const auto values = yaml["imm_q_constant"].as<std::vector<double>>();
    if (values.size() == 3) {
      imm_params.q_constant << values[0], values[1], values[2];
    }
  }
  if (yaml["imm_q_variable"].IsDefined()) {
    const auto values = yaml["imm_q_variable"].as<std::vector<double>>();
    if (values.size() == 3) {
      imm_params.q_variable << values[0], values[1], values[2];
    }
  }
  if (yaml["imm_alpha_decay_slow"].IsDefined()) {
    imm_params.alpha_decay_slow = std::clamp(yaml["imm_alpha_decay_slow"].as<double>(), 0.0, 1.0);
  }
  if (yaml["imm_alpha_decay_constant"].IsDefined()) {
    imm_params.alpha_decay_constant = std::clamp(yaml["imm_alpha_decay_constant"].as<double>(), 0.0, 1.0);
  }
  if (yaml["imm_dt_min"].IsDefined()) {
    imm_params.dt_min = std::max(1e-6, yaml["imm_dt_min"].as<double>());
  }
  if (yaml["imm_dt_max"].IsDefined()) {
    imm_params.dt_max = std::max(imm_params.dt_min, yaml["imm_dt_max"].as<double>());
  }
  spin_imm_.set_params(imm_params);

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
  const std::size_t armors_before_color_filter = armors.size();
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });
  const std::size_t armors_after_color_filter = armors.size();
  if (armors_before_color_filter > 0 && armors_after_color_filter == 0) {
    tools::logger()->warn(
      "[Tracker] All armors filtered by color. enemy_color={} before={} after={}",
      COLORS[enemy_color_], armors_before_color_filter, armors_after_color_filter);
  }

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1280 / 2, 1024 / 2);  // TODO
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
    cv::Point2f img_center(1280 / 2, 1024 / 2);  // TODO
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

  target_.set_jump_params(jump_z_threshold_, jump_yaw_threshold_rad_, jump_confirm_count_);
  target_.set_jump_avg_alpha(jump_avg_alpha_);
  target_.set_process_noise(
    process_noise_linear_normal_, process_noise_angular_normal_, process_noise_linear_outpost_,
    process_noise_angular_outpost_);
  target_.set_measurement_noise(measurement_noise_yaw_, measurement_noise_pitch_);
  if (armor.name == ArmorName::outpost && outpost_jump_fire_cooldown_ > 0.0) {
    target_.set_jump_fire_cooldown(outpost_jump_fire_cooldown_);
  } else {
    target_.set_jump_fire_cooldown(jump_fire_cooldown_);
  }
  target_.set_jump_min_interval(jump_min_interval_);
  if (force_target_angular_velocity_) {
    target_.set_angular_velocity(forced_target_angular_velocity_);
  }
  imm_initialized_ = false;
  update_motion_state(target_, t);

  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);
  if (force_target_angular_velocity_) {
    target_.set_angular_velocity(forced_target_angular_velocity_);
  }

  int name_mismatch_count = 0;
  int type_mismatch_count = 0;
  int candidate_count = 0;

  for (auto & armor : armors) {
    if (armor.name != target_.name) {
      ++name_mismatch_count;
      continue;
    }
    if (armor.type != target_.armor_type) {
      ++type_mismatch_count;
      continue;
    }
    ++candidate_count;

    solver_.solve(armor);
    target_.update(armor);
    update_motion_state(target_, t);
    return true;
  }

  if (!armors.empty()) {
    tools::logger()->warn(
      "[Tracker] No matched armor for target. target(name={}, type={}) armors={} "
      "name_mismatch={} type_mismatch={} candidate={}",
      ARMOR_NAMES[static_cast<int>(target_.name)],
      ARMOR_TYPES[static_cast<int>(target_.armor_type)],
      armors.size(), name_mismatch_count, type_mismatch_count, candidate_count);
  }

  update_motion_state(target_, t);

  return false;
}

void Tracker::update_motion_state(Target & target, std::chrono::steady_clock::time_point t)
{
  if (!motion_state_enabled_) {
    target.set_motion_state(MotionState::static_state);
    target.set_imm_output(target.ekf_x()[7], 0.0);
    return;
  }

  auto ekf_x = target.ekf_x();

  double dt = 0.01;
  if (imm_initialized_) {
    dt = tools::delta_time(t, imm_last_t_);
  }

  double w = ekf_x[7];
  double alpha = 0.0;
  if (imm_enabled_) {
    const double yaw_measure = ekf_x[6];
    if (!imm_initialized_) {
      spin_imm_.reset(yaw_measure, ekf_x[7], 0.0);
      imm_last_w_ = ekf_x[7];
      imm_dw_lpf_ = 0.0;
      imm_initialized_ = true;
    }

    auto imm = spin_imm_.update(yaw_measure, dt);
    w = imm.w;
    alpha = imm.alpha;
  } else if (!imm_initialized_) {
    imm_last_w_ = w;
    imm_dw_lpf_ = 0.0;
    imm_initialized_ = true;
  }

  const double dw_raw = (w - imm_last_w_) / std::max(dt, 1e-3);
  imm_dw_lpf_ = 0.7 * imm_dw_lpf_ + 0.3 * dw_raw;
  imm_last_w_ = w;
  imm_last_t_ = t;

  const double abs_w = std::abs(w);
  const double abs_dw = std::abs(imm_dw_lpf_);

  MotionState state = MotionState::static_state;
  if (abs_w < motion_w_low_) {
    state = MotionState::static_state;
  } else if (abs_dw >= motion_dw_high_) {
    state = MotionState::spin_variable;
  } else if (abs_w < motion_w_high_) {
    state = MotionState::spin_slow_inplace;
  } else {
    state = MotionState::spin_fast_inplace;
  }

  target.set_motion_state(state);
  target.set_imm_output(w, alpha);
}

}  // namespace auto_aim