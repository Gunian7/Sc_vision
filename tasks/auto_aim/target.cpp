#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  armor_num_(armor_num),
  switch_count_(0),
  update_count_(0),
  is_switch_(false),
  is_converged_(false),
  height_init_done_(true),
  outpost_all_ids_seen_(false),
  outpost_height_min_gap_(0.05),
  outpost_match_z_gate_(0.16),
  outpost_match_z_penalty_scale_(25.0),
  match_gate_tracked_(12.0),
  match_gate_init_(24.0),
  last_jump_dir_(0),
  has_jump_time_(false),
  jump_z_threshold_(0.02),
  jump_yaw_threshold_rad_(40.0 / 57.3),
  jump_confirm_count_(1),
  jump_pending_dir_(0),
  jump_pending_count_(0),
  jump_avg_alpha_(1.0),
  jump_fire_cooldown_(0.0),
  jump_min_interval_(0.0),
  process_noise_linear_normal_(100.0),
  process_noise_angular_normal_(400.0),
  process_noise_linear_outpost_(10.0),
  process_noise_angular_outpost_(0.1),
  measurement_noise_yaw_(2e-3),
  measurement_noise_pitch_(2e-3),
  motion_state_(MotionState::static_state),
  imm_w_(0.0),
  imm_alpha_(0.0),
  t_(t)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  height_offsets_.fill(0.0);
  outpost_seen_ids_.clear();
  if (name == ArmorName::outpost && armor_num_ == 3) {
    height_init_done_ = false;
    height_init_start_ = t;
    for (auto & samples : height_samples_) {
      samples.clear();
    }
  }
  jump_avg_z_.fill(0.0);
  jump_avg_inited_.fill(false);

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];
  auto height_step = 0.0;

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0(11);
  x0 << center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, height_step;  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0(11);
  x0 << x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h;
  Eigen::VectorXd P0_dig(11);
  P0_dig << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  height_offsets_.fill(0.0);
  outpost_seen_ids_.clear();
  height_init_done_ = true;
  outpost_all_ids_seen_ = false;
  outpost_height_min_gap_ = 0.05;
  outpost_match_z_gate_ = 0.16;
  outpost_match_z_penalty_scale_ = 25.0;
  match_gate_tracked_ = 12.0;
  match_gate_init_ = 24.0;
  last_jump_dir_ = 0;
  has_jump_time_ = false;
  jump_z_threshold_ = 0.02;
  jump_yaw_threshold_rad_ = 40.0 / 57.3;
  jump_confirm_count_ = 1;
  jump_pending_dir_ = 0;
  jump_pending_count_ = 0;
  jump_avg_alpha_ = 1.0;
  jump_avg_z_.fill(0.0);
  jump_avg_inited_.fill(false);
  jump_fire_cooldown_ = 0.0;
  jump_min_interval_ = 0.0;
  process_noise_linear_normal_ = 100.0;
  process_noise_angular_normal_ = 400.0;
  process_noise_linear_outpost_ = 10.0;
  process_noise_angular_outpost_ = 0.1;
  measurement_noise_yaw_ = 2e-3;
  measurement_noise_pitch_ = 2e-3;
  motion_state_ = MotionState::static_state;
  imm_w_ = 0.0;
  imm_alpha_ = 0.0;
  for (auto & samples : height_samples_) {
    samples.clear();
  }

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = process_noise_linear_outpost_;   // 前哨站加速度方差
    v2 = process_noise_angular_outpost_;  // 前哨站角加速度方差
  } else {
    v1 = process_noise_linear_normal_;   // 加速度方差
    v2 = process_noise_angular_normal_;  // 角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);
}

void Target::update(const Armor & armor)
{
  int id = match_armor_id(armor);
  if (id < 0) {
    id = 0;
  }

  const std::vector<Eigen::Vector4d> xyza_list = armor_xyza_list();
  update_outpost_seen_ids(id);
  update_outpost_height_samples(armor, id);

  if (id < static_cast<int>(xyza_list.size())) {
    auto current_z = xyza_list[id][2];
    if (!jump_avg_inited_[id]) {
      jump_avg_z_[id] = current_z;
      jump_avg_inited_[id] = true;
    } else {
      jump_avg_z_[id] = jump_avg_alpha_ * current_z + (1.0 - jump_avg_alpha_) * jump_avg_z_[id];
    }
  }

  update_switch_state(id, xyza_list);
  update_ypda(armor, id);
}

bool Target::match_and_update(const std::vector<Armor> & armors)
{
  int best_armor_index = -1;
  int best_id = -1;
  double best_d2 = std::numeric_limits<double>::infinity();

  for (int i = 0; i < static_cast<int>(armors.size()); ++i) {
    double d2 = std::numeric_limits<double>::infinity();
    int id = match_armor_id(armors[i], &d2);
    if (id < 0) {
      continue;
    }
    if (d2 < best_d2) {
      best_d2 = d2;
      best_id = id;
      best_armor_index = i;
    }
  }

  if (best_armor_index < 0 || best_id < 0) {
    return false;
  }

  const auto & armor = armors[best_armor_index];
  const std::vector<Eigen::Vector4d> xyza_list = armor_xyza_list();
  update_outpost_seen_ids(best_id);
  update_outpost_height_samples(armor, best_id);

  if (best_id < static_cast<int>(xyza_list.size())) {
    auto current_z = xyza_list[best_id][2];
    if (!jump_avg_inited_[best_id]) {
      jump_avg_z_[best_id] = current_z;
      jump_avg_inited_[best_id] = true;
    } else {
      jump_avg_z_[best_id] =
        jump_avg_alpha_ * current_z + (1.0 - jump_avg_alpha_) * jump_avg_z_[best_id];
    }
  }

  update_switch_state(best_id, xyza_list);
  update_ypda(armor, best_id);
  return true;
}

int Target::match_armor_id(const Armor & armor, double * best_d2) const
{
  int best_id = -1;
  double min_d2 = std::numeric_limits<double>::infinity();
  const Eigen::Vector4d z = measurement_from_armor(armor);
  const Eigen::MatrixXd R = measurement_noise_matrix(armor);
  const bool use_outpost_z_match =
    (name == ArmorName::outpost && armor_num_ == 3 && height_init_done_);

  for (int id = 0; id < armor_num_; ++id) {
    const Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
    const Eigen::Vector4d z_pred = predicted_measurement(ekf_.x, id);
    const Eigen::VectorXd residual = measurement_subtract(z, z_pred);
    const Eigen::MatrixXd S = H * ekf_.P * H.transpose() + R;
    const Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
      continue;
    }
    double d2 = residual.transpose() * ldlt.solve(residual);
    if (!std::isfinite(d2)) {
      continue;
    }

    if (use_outpost_z_match) {
      const double z_pred_world = h_armor_xyz(ekf_.x, id)[2];
      const double z_residual = armor.xyz_in_world[2] - z_pred_world;
      const double abs_z_residual = std::abs(z_residual);
      if (abs_z_residual > outpost_match_z_gate_) {
        continue;
      }
      d2 += outpost_match_z_penalty_scale_ * abs_z_residual * abs_z_residual;
    }

    if (d2 < min_d2) {
      min_d2 = d2;
      best_id = id;
    }
  }

  if (best_d2 != nullptr) {
    *best_d2 = min_d2;
  }

  if (best_id < 0) {
    return -1;
  }

  const bool use_init_gate =
    (name == ArmorName::outpost && armor_num_ == 3 && !outpost_all_ids_seen_);
  const double gate = use_init_gate ? match_gate_init_ : match_gate_tracked_;
  return min_d2 <= gate ? best_id : -1;
}

Eigen::Vector4d Target::measurement_from_armor(const Armor & armor) const
{
  return {armor.ypd_in_world[0], armor.ypd_in_world[1], armor.ypd_in_world[2], armor.ypr_in_world[0]};
}

Eigen::MatrixXd Target::measurement_noise_matrix(const Armor & armor) const
{
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::VectorXd R_dig(4);
  R_dig << measurement_noise_yaw_, measurement_noise_pitch_,
    log(std::abs(delta_angle) + 1) + 0.5,
    log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 4.5e-2;
  return R_dig.asDiagonal();
}

Eigen::Vector4d Target::predicted_measurement(const Eigen::VectorXd & x, int id) const
{
  Eigen::VectorXd xyz = h_armor_xyz(x, id);
  Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  return {ypd[0], ypd[1], ypd[2], angle};
}

Eigen::VectorXd Target::measurement_subtract(
  const Eigen::VectorXd & a, const Eigen::VectorXd & b) const
{
  Eigen::VectorXd c = a - b;
  c[0] = tools::limit_rad(c[0]);
  c[1] = tools::limit_rad(c[1]);
  c[3] = tools::limit_rad(c[3]);
  return c;
}

void Target::update_outpost_seen_ids(int id)
{
  if (name != ArmorName::outpost || armor_num_ != 3 || id < 0 || id >= armor_num_) {
    return;
  }
  outpost_seen_ids_.insert(id);
  if (static_cast<int>(outpost_seen_ids_.size()) >= armor_num_) {
    outpost_all_ids_seen_ = true;
  }
}

double Target::robust_height_stat(const std::vector<double> & samples) const
{
  if (samples.empty()) {
    return 0.0;
  }

  std::vector<double> sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  const size_t n = sorted.size();
  if (n % 2 == 1) {
    return sorted[n / 2];
  }
  return 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
}

void Target::update_outpost_height_samples(const Armor & armor, int id)
{
  if (
    name != ArmorName::outpost || armor_num_ != 3 || height_init_done_ || id < 0 ||
    id >= armor_num_)
  {
    return;
  }

  height_samples_[id].push_back(armor.xyz_in_world[2]);
  auto elapsed = std::chrono::duration<double>(t_ - height_init_start_).count();
  if (elapsed < 2.5) {
    return;
  }

  if (!outpost_all_ids_seen_) {
    tools::logger()->debug(
      "[Target] Outpost height init waiting for all ids, seen={}", outpost_seen_ids_.size());
    return;
  }

  constexpr size_t kMinSamplesPerId = 3;
  for (int i = 0; i < 3; ++i) {
    if (height_samples_[i].size() < kMinSamplesPerId) {
      tools::logger()->debug(
        "[Target] Outpost height init waiting for samples: id{} has {}", i,
        height_samples_[i].size());
      return;
    }
  }

  std::array<double, 3> stats;
  for (int i = 0; i < 3; ++i) {
    stats[i] = robust_height_stat(height_samples_[i]);
  }

  std::array<int, 3> order{0, 1, 2};
  std::sort(order.begin(), order.end(), [&](int a, int b) { return stats[a] < stats[b]; });

  const double low_mid_gap = stats[order[1]] - stats[order[0]];
  const double mid_high_gap = stats[order[2]] - stats[order[1]];
  if (low_mid_gap < outpost_height_min_gap_ || mid_high_gap < outpost_height_min_gap_) {
    tools::logger()->debug(
      "[Target] Outpost height init waiting for clearer separation: "
      "median_z=[{:.3f}, {:.3f}, {:.3f}], gaps=[{:.3f}, {:.3f}], min_gap={:.3f}",
      stats[0], stats[1], stats[2], low_mid_gap, mid_high_gap, outpost_height_min_gap_);
    return;
  }

  height_offsets_.fill(0.0);
  height_offsets_[order[0]] = -0.1;
  height_offsets_[order[1]] = 0.0;
  height_offsets_[order[2]] = 0.1;
  height_init_done_ = true;
  tools::logger()->info(
    "[Target] Outpost height offsets fixed by robust id order: "
    "id0={:.3f}, id1={:.3f}, id2={:.3f}, median_z=[{:.3f}, {:.3f}, {:.3f}], "
    "gaps=[{:.3f}, {:.3f}]",
    height_offsets_[0], height_offsets_[1], height_offsets_[2], stats[0], stats[1], stats[2],
    low_mid_gap, mid_high_gap);
}

void Target::update_switch_state(int id, const std::vector<Eigen::Vector4d> & xyza_list)
{
  if (id != last_id) {
    int candidate_dir = 0;
    double delta_yaw = 0.0;
    double delta_z = 0.0;
    bool is_outpost_z_mode = (name == ArmorName::outpost && armor_num_ == 3 && height_init_done_);
    if (is_outpost_z_mode) {
      auto prev_offset = height_offsets_[last_id];
      auto new_offset = height_offsets_[id];
      delta_z = new_offset - prev_offset;
      if (std::abs(delta_z) >= jump_z_threshold_) {
        candidate_dir = (delta_z > 0) ? 1 : -1;
      }
    } else {
      if (static_cast<int>(xyza_list.size()) > std::max(id, last_id)) {
        auto prev_yaw = xyza_list[last_id][3];
        auto new_yaw = xyza_list[id][3];
        delta_yaw = tools::limit_rad(new_yaw - prev_yaw);
      }
      if (std::abs(delta_yaw) >= jump_yaw_threshold_rad_) {
        candidate_dir = (delta_yaw > 0) ? 1 : -1;
      }
    }

    if (candidate_dir == 0) {
      if (is_outpost_z_mode) {
        tools::logger()->info("[Target] Jump detected: outpost dz too small (dz={:.3f} m)", delta_z);
      } else {
        tools::logger()->info(
          "[Target] Jump detected: yaw change too small (dyaw={:.2f} deg)", delta_yaw * 57.3);
      }
    }

    if (candidate_dir != 0) {
      if (candidate_dir == jump_pending_dir_) {
        jump_pending_count_++;
      } else {
        jump_pending_dir_ = candidate_dir;
        jump_pending_count_ = 1;
      }

      if (jump_pending_count_ >= jump_confirm_count_) {
        auto can_confirm = true;
        if (has_jump_time_ && jump_min_interval_ > 0.0) {
          auto since_last = std::chrono::duration<double>(t_ - last_jump_time_).count();
          if (since_last >= 0.0 && since_last < jump_min_interval_) {
            can_confirm = false;
          }
        }
        if (can_confirm) {
          last_jump_dir_ = candidate_dir;
          last_jump_time_ = t_;
          has_jump_time_ = true;
          if (is_outpost_z_mode) {
            if (candidate_dir < 0) {
              tools::logger()->info(
                "[Target] Jump confirmed by outpost z: high -> low (dz={:.3f} m)", delta_z);
            } else {
              tools::logger()->info(
                "[Target] Jump confirmed by outpost z: low -> high (dz={:.3f} m)", delta_z);
            }
          } else {
            if (candidate_dir < 0) {
              tools::logger()->info(
                "[Target] Jump confirmed by yaw: ccw -> cw (dyaw={:.2f} deg)", delta_yaw * 57.3);
            } else {
              tools::logger()->info(
                "[Target] Jump confirmed by yaw: cw -> ccw (dyaw={:.2f} deg)", delta_yaw * 57.3);
            }
          }
        }
        jump_pending_count_ = 0;
        jump_pending_dir_ = 0;
      }
    }
  }

  if (id != 0) {
    jumped = true;
  }

  is_switch_ = (id != last_id);

  if (is_switch_) {
    switch_count_++;
  }

  last_id = id;
  update_count_++;
}

void Target::update_ypda(const Armor & armor, int id)
{
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  Eigen::MatrixXd R = measurement_noise_matrix(armor);

  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    return predicted_measurement(x, id);
  };

  auto z_subtract = [&](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    return measurement_subtract(a, b);
  };

  Eigen::VectorXd z = measurement_from_armor(armor);
  ekf_.update(z, H, R, h, z_subtract);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = x[4];
  if (use_l_h) {
    armor_z += x[10];
  } else if (armor_num_ == 3 && name == ArmorName::outpost && height_init_done_) {
    armor_z += height_offsets_[id];
  }

  return {armor_x, armor_y, armor_z};
}

// 观测雅可比矩阵
Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

bool Target::outpost_height_ready() const { return height_init_done_; }

int Target::last_jump_dir() const { return last_jump_dir_; }

bool Target::has_jump_time() const { return has_jump_time_; }

std::chrono::steady_clock::time_point Target::last_jump_time() const { return last_jump_time_; }

void Target::set_jump_params(double z_threshold, double yaw_threshold_rad, int confirm_count)
{
  jump_z_threshold_ = std::max(0.0, z_threshold);
  jump_yaw_threshold_rad_ = std::max(0.0, yaw_threshold_rad);
  jump_confirm_count_ = std::max(1, confirm_count);
}

void Target::set_jump_avg_alpha(double alpha)
{
  jump_avg_alpha_ = std::clamp(alpha, 0.0, 1.0);
}

void Target::set_jump_fire_cooldown(double seconds)
{
  jump_fire_cooldown_ = std::max(0.0, seconds);
}

void Target::set_jump_min_interval(double seconds)
{
  jump_min_interval_ = std::max(0.0, seconds);
}

void Target::set_process_noise(
  double linear_acc_normal, double angular_acc_normal, double linear_acc_outpost,
  double angular_acc_outpost)
{
  process_noise_linear_normal_ = std::max(0.0, linear_acc_normal);
  process_noise_angular_normal_ = std::max(0.0, angular_acc_normal);
  process_noise_linear_outpost_ = std::max(0.0, linear_acc_outpost);
  process_noise_angular_outpost_ = std::max(0.0, angular_acc_outpost);
}

void Target::set_measurement_noise(double yaw_noise, double pitch_noise)
{
  measurement_noise_yaw_ = std::max(1e-9, yaw_noise);
  measurement_noise_pitch_ = std::max(1e-9, pitch_noise);
}

void Target::set_match_gates(double tracked_gate, double init_gate)
{
  match_gate_tracked_ = std::max(1e-6, tracked_gate);
  match_gate_init_ = std::max(match_gate_tracked_, init_gate);
}

bool Target::in_jump_fire_cooldown(std::chrono::steady_clock::time_point t) const
{
  if (!has_jump_time_ || jump_fire_cooldown_ <= 0.0) return false;
  auto age = std::chrono::duration<double>(t - last_jump_time_).count();
  return age >= 0.0 && age <= jump_fire_cooldown_;
}

void Target::set_angular_velocity(double angular_velocity) { ekf_.x[7] = angular_velocity; }

}  // namespace auto_aim
