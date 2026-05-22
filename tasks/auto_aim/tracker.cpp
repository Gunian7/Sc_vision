#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>
#include <numeric>
#include <tuple>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
constexpr int kIdxVx = 1;
constexpr int kIdxVy = 3;
constexpr int kIdxVz = 5;
constexpr int kIdxYaw = 6;
constexpr int kIdxW = 7;
constexpr int kIdxAlpha = 11;
constexpr double kDefaultMotionDt = 0.01;
constexpr size_t kConstantSpinModelIndex = 1;

bool should_mark_bad_converge(const Target & target)
{
  const auto & ekf = target.ekf();
  const auto sample_count = ekf.recent_nis_failures.size();
  const auto min_samples = std::min<size_t>(ekf.window_size, 20);
  if (sample_count < min_samples) {
    return false;
  }

  const int recent_failures =
    std::accumulate(ekf.recent_nis_failures.begin(), ekf.recent_nis_failures.end(), 0);
  const double fail_rate = static_cast<double>(recent_failures) / static_cast<double>(sample_count);
  constexpr double fail_rate_threshold = 0.4;

  if (recent_failures >= static_cast<int>(fail_rate_threshold * ekf.window_size)) {
    tools::logger()->warn(
      "[Target] Bad converge: recent_nis_failures={}/{}, fail_rate={:.3f}, last_nis={:.3f}, "
      "window_size={}",
      recent_failures, sample_count, fail_rate, ekf.last_nis, ekf.window_size);
    return true;
  }

  return false;
}

bool select_best_candidate(
  const Target & target, const std::vector<Armor> & candidates, int * best_armor_index,
  int * best_id)
{
  if (best_armor_index == nullptr || best_id == nullptr) {
    return false;
  }

  *best_armor_index = -1;
  *best_id = -1;
  double best_d2 = std::numeric_limits<double>::infinity();

  for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
    double d2 = std::numeric_limits<double>::infinity();
    const int id = target.match_armor_id(candidates[i], &d2);
    if (id < 0) {
      continue;
    }
    if (d2 < best_d2) {
      best_d2 = d2;
      *best_armor_index = i;
      *best_id = id;
    }
  }

  return *best_armor_index >= 0 && *best_id >= 0;
}

void configure_imm_for_target(SpinIMM * spin_imm, const Target & target)
{
  if (spin_imm == nullptr) {
    return;
  }

  if (target.name == ArmorName::outpost) {
    spin_imm->set_model_lock(kConstantSpinModelIndex);
  } else {
    spin_imm->clear_model_lock();
  }
}
}  // namespace

Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth},
  plotter_{config_path},
  imm_enabled_(true),
  motion_state_enabled_(true),
  motion_w_low_(1.2),
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
  target_match_gate_tracked_ = 12.0;
  target_match_gate_init_ = 24.0;
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
  if (yaml["target_match_gate_tracked"].IsDefined()) {
    target_match_gate_tracked_ = yaml["target_match_gate_tracked"].as<double>();
  }
  if (yaml["target_match_gate_init"].IsDefined()) {
    target_match_gate_init_ = yaml["target_match_gate_init"].as<double>();
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
  if (yaml["motion_dw_high"].IsDefined()) {
    motion_dw_high_ = yaml["motion_dw_high"].as<double>();
  }
  SpinIMM::Params imm_params;
  imm_params.transition << 0.93, 0.05, 0.02, 0.04, 0.93, 0.03, 0.03, 0.07, 0.90;
  imm_params.r_yaw = 2e-3;
  imm_params.q_v_slow = 8e-2;
  imm_params.q_v_constant = 2e-2;
  imm_params.q_v_variable = 2e-1;
  imm_params.q_alpha_slow = 1e-1;
  imm_params.q_alpha_constant = 5e-2;
  imm_params.q_alpha_variable = 8e-1;
  imm_params.alpha_decay_slow = 0.2;
  imm_params.alpha_decay_constant = 0.5;
  imm_params.dt_min = 1e-3;
  imm_params.dt_max = 0.2;
  imm_params.mu_min = 1e-4;

  if (yaml["imm_transition"].IsDefined()) {
    const auto values = yaml["imm_transition"].as<std::vector<double>>();
    if (values.size() == 9) {
      imm_params.transition << values[0], values[1], values[2], values[3], values[4], values[5],
        values[6], values[7], values[8];
    }
  }
  if (yaml["imm_r_yaw"].IsDefined()) {
    imm_params.r_yaw = std::max(1e-9, yaml["imm_r_yaw"].as<double>());
  }
  if (yaml["imm_q_v_slow"].IsDefined()) {
    imm_params.q_v_slow = yaml["imm_q_v_slow"].as<double>();
  }
  if (yaml["imm_q_v_constant"].IsDefined()) {
    imm_params.q_v_constant = yaml["imm_q_v_constant"].as<double>();
  }
  if (yaml["imm_q_v_variable"].IsDefined()) {
    imm_params.q_v_variable = yaml["imm_q_v_variable"].as<double>();
  }
  if (yaml["imm_q_alpha_slow"].IsDefined()) {
    imm_params.q_alpha_slow = yaml["imm_q_alpha_slow"].as<double>();
  }
  if (yaml["imm_q_alpha_constant"].IsDefined()) {
    imm_params.q_alpha_constant = yaml["imm_q_alpha_constant"].as<double>();
  }
  if (yaml["imm_q_alpha_variable"].IsDefined()) {
    imm_params.q_alpha_variable = yaml["imm_q_alpha_variable"].as<double>();
  }
  if (yaml["imm_alpha_decay_slow"].IsDefined()) {
    imm_params.alpha_decay_slow = std::clamp(yaml["imm_alpha_decay_slow"].as<double>(), 0.0, 1.0);
  }
  if (yaml["imm_alpha_decay_constant"].IsDefined()) {
    imm_params.alpha_decay_constant =
      std::clamp(yaml["imm_alpha_decay_constant"].as<double>(), 0.0, 1.0);
  }
  if (yaml["imm_dt_min"].IsDefined()) {
    imm_params.dt_min = std::max(1e-6, yaml["imm_dt_min"].as<double>());
  }
  if (yaml["imm_dt_max"].IsDefined()) {
    imm_params.dt_max = std::max(imm_params.dt_min, yaml["imm_dt_max"].as<double>());
  }
  if (yaml["imm_mu_min"].IsDefined()) {
    imm_params.mu_min = yaml["imm_mu_min"].as<double>();
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
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 同优先级下优先选择靠近图像中心的装甲板
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
  if (state_ != "lost" && should_mark_bad_converge(target_)) {
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

  // 收敛效果检测：
  if (state_ != "lost" && should_mark_bad_converge(target_)) {
    state_ = "lost";
    return {switch_target, {}};
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
    Eigen::VectorXd P0_dig(12);
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1, 1;
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig(12);
    P0_dig << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 1e-4, 1e-4;
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig(12);
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0, 0;
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig(12);
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1, 1;
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  target_.set_jump_params(jump_z_threshold_, jump_yaw_threshold_rad_, jump_confirm_count_);
  target_.set_jump_avg_alpha(jump_avg_alpha_);
  target_.set_process_noise(
    process_noise_linear_normal_, process_noise_angular_normal_, process_noise_linear_outpost_,
    process_noise_angular_outpost_);
  target_.set_measurement_noise(measurement_noise_yaw_, measurement_noise_pitch_);
  target_.set_match_gates(target_match_gate_tracked_, target_match_gate_init_);
  if (armor.name == ArmorName::outpost && outpost_jump_fire_cooldown_ > 0.0) {
    target_.set_jump_fire_cooldown(outpost_jump_fire_cooldown_);
  } else {
    target_.set_jump_fire_cooldown(jump_fire_cooldown_);
  }
  target_.set_jump_min_interval(jump_min_interval_);
  if (force_target_angular_velocity_) {
    target_.set_angular_velocity(forced_target_angular_velocity_);
  }

  if (imm_enabled_) {
    configure_imm_for_target(&spin_imm_, target_);
    const double init_yaw = target_.ekf_x()[kIdxYaw];
    const double init_w = target_.ekf_x()[kIdxW];
    const double init_alpha = target_.ekf_x()[kIdxAlpha];
    const double init_P_yaw = target_.ekf().P(kIdxYaw, kIdxYaw);
    const double init_P_w = target_.ekf().P(kIdxW, kIdxW);
    const double init_P_alpha = target_.ekf().P(kIdxAlpha, kIdxAlpha);
    spin_imm_.initialize(init_yaw, init_w, init_alpha, init_P_yaw, init_P_w, init_P_alpha);
    imm_initialized_ = true;
  } else {
    spin_imm_.reset();
    imm_initialized_ = false;
  }

  imm_last_w_ = target_.ekf_x()[kIdxW];
  imm_dw_lpf_ = 0.0;
  imm_last_t_ = t;

  update_motion_state(target_, t);
  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  std::vector<Armor> candidates;
  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;

    solver_.solve(armor);
    candidates.push_back(armor);
  }

  if (!imm_enabled_) {
    target_.predict(t);
    if (force_target_angular_velocity_) {
      target_.set_angular_velocity(forced_target_angular_velocity_);
    }

    bool found = false;
    if (!candidates.empty()) {
      found = target_.match_and_update(candidates);
    }

    update_motion_state(target_, t);
    return found;
  }

  configure_imm_for_target(&spin_imm_, target_);

  if (!spin_imm_.initialized()) {
    const double init_yaw = target_.ekf_x()[kIdxYaw];
    const double init_w = target_.ekf_x()[kIdxW];
    const double init_alpha = target_.ekf_x()[kIdxAlpha];
    const double init_P_yaw = target_.ekf().P(kIdxYaw, kIdxYaw);
    const double init_P_w = target_.ekf().P(kIdxW, kIdxW);
    const double init_P_alpha = target_.ekf().P(kIdxAlpha, kIdxAlpha);
    spin_imm_.initialize(init_yaw, init_w, init_alpha, init_P_yaw, init_P_w, init_P_alpha);
    imm_initialized_ = true;
    imm_last_w_ = target_.ekf_x()[kIdxW];
    imm_dw_lpf_ = 0.0;
    imm_last_t_ = t;
  }

  const double dt = imm_initialized_ ? tools::delta_time(t, imm_last_t_) : kDefaultMotionDt;
  spin_imm_.predict(dt);

  // 复用 Target 内部时间与状态机辅助逻辑
  target_.predict(t);

  bool found = false;
  int best_armor_index = -1;
  int best_id = -1;
  if (select_best_candidate(target_, candidates, &best_armor_index, &best_id)) {
    const auto & armor = candidates[best_armor_index];
    target_.apply_measurement_bookkeeping(armor, best_id);

    // 直接观测补偿装甲板 id 后的旋转中心 yaw 位置
    const double armor_num = static_cast<double>(target_.armor_num());
    const double observed_center_yaw = tools::limit_rad(
      armor.ypr_in_world[0] - best_id * 2.0 * M_PI / armor_num);

    found = spin_imm_.update(observed_center_yaw, measurement_noise_yaw_);
    if (found) {
      // IMM 的 yaw/v_yaw/alpha_yaw 直接输出给下游，不写回 EKF
      target_.set_imm_output(spin_imm_.yaw(), spin_imm_.v_yaw(), spin_imm_.alpha_yaw());
    }
  }

  if (force_target_angular_velocity_) {
    target_.set_angular_velocity(forced_target_angular_velocity_);
    const double init_yaw = target_.ekf_x()[kIdxYaw];
    const double init_w = target_.ekf_x()[kIdxW];
    const double init_alpha = target_.ekf_x()[kIdxAlpha];
    const double init_P_yaw = target_.ekf().P(kIdxYaw, kIdxYaw);
    const double init_P_w = target_.ekf().P(kIdxW, kIdxW);
    const double init_P_alpha = target_.ekf().P(kIdxAlpha, kIdxAlpha);
    spin_imm_.initialize(init_yaw, init_w, init_alpha, init_P_yaw, init_P_w, init_P_alpha);
  }

  update_motion_state(target_, t);
  return found;
}

void Tracker::update_motion_state(Target & target, std::chrono::steady_clock::time_point t)
{
  std::array<double, SpinIMM::kModelCount> model_probs{};
  std::array<double, SpinIMM::kModelCount> model_ws{};
  std::array<double, SpinIMM::kModelCount> model_alphas{};

  Eigen::VectorXd fused_state = target.ekf_x();
  SpinModel spin_state = SpinModel::slow;

  if (imm_enabled_ && spin_imm_.initialized()) {
    // Use IMM's fused v_yaw and alpha_yaw directly
    const double imm_v = spin_imm_.v_yaw();
    const double imm_alpha = spin_imm_.alpha_yaw();
    fused_state[kIdxW] = imm_v;
    fused_state[kIdxAlpha] = imm_alpha;

    model_probs = spin_imm_.getModelProbs();
    model_ws = spin_imm_.getModelAngularVelocitys();
    model_alphas = spin_imm_.getModelAngularAccelerations();

    const auto best_model_it = std::max_element(model_probs.begin(), model_probs.end());
    const size_t best_model_index = std::distance(model_probs.begin(), best_model_it);
    switch (best_model_index) {
      case 0:
        spin_state = SpinModel::slow;
        break;
      case 1:
        spin_state = SpinModel::constant;
        break;
      default:
        spin_state = SpinModel::variable;
        break;
    }
  } else if (motion_state_enabled_) {
    const double w_for_fallback = fused_state.size() > kIdxW ? fused_state[kIdxW] : 0.0;
    const double alpha_for_fallback = fused_state.size() > kIdxAlpha ? fused_state[kIdxAlpha] : 0.0;
    const double abs_w = std::abs(w_for_fallback);
    const double abs_alpha = std::abs(alpha_for_fallback);

    if (abs_alpha >= motion_dw_high_) {
      spin_state = SpinModel::variable;
    } else if (abs_w >= motion_w_low_) {
      spin_state = SpinModel::constant;
    } else {
      spin_state = SpinModel::slow;
    }
  }

  double w = 0.0;
  double alpha = 0.0;
  double linear_speed = 0.0;
  if (fused_state.size() > kIdxW) {
    w = fused_state[kIdxW];
  }
  if (fused_state.size() > kIdxAlpha) {
    alpha = fused_state[kIdxAlpha];
  }
  if (fused_state.size() > kIdxVz) {
    linear_speed = std::sqrt(
      fused_state[kIdxVx] * fused_state[kIdxVx] + fused_state[kIdxVy] * fused_state[kIdxVy] +
      fused_state[kIdxVz] * fused_state[kIdxVz]);
  }

  const double dt = imm_initialized_ ? tools::delta_time(t, imm_last_t_) : kDefaultMotionDt;
  if (!imm_initialized_) {
    imm_last_w_ = w;
    imm_dw_lpf_ = 0.0;
    imm_initialized_ = true;
  }

  const double dw_raw = (w - imm_last_w_) / std::max(dt, 1e-3);
  imm_dw_lpf_ = 0.7 * imm_dw_lpf_ + 0.3 * dw_raw;
  imm_last_w_ = w;
  imm_last_t_ = t;

  target.set_spin_state(spin_state);
  target.set_linear_speed(linear_speed);
  target.set_imm_output(spin_imm_.yaw(), w, alpha);

  nlohmann::json plot_json;
  plot_json["imm_enabled"] = imm_enabled_;
  plot_json["spin_state"] = static_cast<int>(spin_state);
  plot_json["linear_speed"] = linear_speed;
  plot_json["fused_w"] = w;
  plot_json["fused_alpha"] = alpha;
  plot_json["ekf_w"] = target.ekf_x().size() > kIdxW ? target.ekf_x()[kIdxW] : 0.0;
  plot_json["ekf_alpha"] = target.ekf_x().size() > kIdxAlpha ? target.ekf_x()[kIdxAlpha] : 0.0;
  plot_json["imm_dw_lpf"] = imm_dw_lpf_;
  plot_json["model_prob_slow"] = model_probs[0];
  plot_json["model_prob_constant"] = model_probs[1];
  plot_json["model_prob_variable"] = model_probs[2];
  plot_json["model_w_slow"] = model_ws[0];
  plot_json["model_w_constant"] = model_ws[1];
  plot_json["model_w_variable"] = model_ws[2];
  plot_json["model_alpha_slow"] = model_alphas[0];
  plot_json["model_alpha_constant"] = model_alphas[1];
  plot_json["model_alpha_variable"] = model_alphas[2];
  plotter_.plot(plot_json);
}

}  // namespace auto_aim
