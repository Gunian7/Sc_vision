#include "imm.hpp"

#include <algorithm>
#include <cmath>

namespace auto_aim
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

SpinIMM::SpinIMM() : initialized_(false)
{
  params_.transition << 0.93, 0.05, 0.02, 0.04, 0.93, 0.03, 0.03, 0.07, 0.90;
  params_.r_yaw = 2e-3;
  params_.q_slow << 1e-4, 8e-2, 1e-1;
  params_.q_constant << 1e-4, 2e-2, 5e-2;
  params_.q_variable << 2e-4, 2e-1, 8e-1;
  params_.alpha_decay_slow = 0.2;
  params_.alpha_decay_constant = 0.5;
  params_.dt_min = 1e-3;
  params_.dt_max = 0.2;

  mu_ = {0.34, 0.33, 0.33};

  for (auto & model : models_) {
    model.x.setZero();
    model.P = Eigen::Matrix3d::Identity() * 0.1;
    model.Q = Eigen::Matrix3d::Identity() * 1e-3;
  }

  models_[0].Q.diagonal() = params_.q_slow;
  models_[1].Q.diagonal() = params_.q_constant;
  models_[2].Q.diagonal() = params_.q_variable;
}

void SpinIMM::set_params(const Params & params)
{
  params_ = params;
  models_[0].Q.diagonal() = params_.q_slow;
  models_[1].Q.diagonal() = params_.q_constant;
  models_[2].Q.diagonal() = params_.q_variable;
}

double SpinIMM::normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

Eigen::Vector3d SpinIMM::blend_state(
  const std::array<Eigen::Vector3d, 3> & states, const std::array<double, 3> & weights)
{
  Eigen::Vector3d mixed = Eigen::Vector3d::Zero();
  double c = 0.0;
  double s = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    mixed[1] += weights[i] * states[i][1];
    mixed[2] += weights[i] * states[i][2];
    c += weights[i] * std::cos(states[i][0]);
    s += weights[i] * std::sin(states[i][0]);
  }
  mixed[0] = std::atan2(s, c);
  return mixed;
}

void SpinIMM::reset(double yaw, double w, double alpha)
{
  initialized_ = true;
  mu_ = {0.34, 0.33, 0.33};

  for (auto & model : models_) {
    model.x << normalize_angle(yaw), w, alpha;
    model.P = Eigen::Matrix3d::Identity() * 0.05;
  }
}

SpinIMM::Output SpinIMM::update(double measured_yaw, double dt)
{
  measured_yaw = normalize_angle(measured_yaw);
  if (!initialized_) {
    reset(measured_yaw, 0.0, 0.0);
  }

  dt = std::clamp(dt, params_.dt_min, params_.dt_max);

  std::array<double, 3> c{};
  for (size_t j = 0; j < 3; ++j) {
    for (size_t i = 0; i < 3; ++i) {
      c[j] += params_.transition(i, j) * mu_[i];
    }
    c[j] = std::max(c[j], 1e-9);
  }

  std::array<Eigen::Vector3d, 3> mixed_x{};
  std::array<Eigen::Matrix3d, 3> mixed_p{};
  for (size_t j = 0; j < 3; ++j) {
    std::array<double, 3> mix_w{};
    for (size_t i = 0; i < 3; ++i) {
      mix_w[i] = params_.transition(i, j) * mu_[i] / c[j];
    }

    std::array<Eigen::Vector3d, 3> states{models_[0].x, models_[1].x, models_[2].x};
    mixed_x[j] = blend_state(states, mix_w);

    Eigen::Matrix3d p = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < 3; ++i) {
      Eigen::Vector3d dx = models_[i].x - mixed_x[j];
      dx[0] = normalize_angle(dx[0]);
      p += mix_w[i] * (models_[i].P + dx * dx.transpose());
    }
    mixed_p[j] = p;
  }

  std::array<double, 3> likelihood{};
  for (size_t j = 0; j < 3; ++j) {
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    if (j == 0) {
      F(0, 1) = dt;
      F(0, 2) = 0.0;
      F(1, 2) = 0.0;
      F(2, 2) = params_.alpha_decay_slow;
    } else if (j == 1) {
      F(0, 1) = dt;
      F(0, 2) = 0.0;
      F(1, 2) = 0.0;
      F(2, 2) = params_.alpha_decay_constant;
    } else {
      F(0, 1) = dt;
      F(0, 2) = 0.5 * dt * dt;
      F(1, 2) = dt;
    }

    Eigen::Vector3d x_pred = F * mixed_x[j];
    x_pred[0] = normalize_angle(x_pred[0]);
    Eigen::Matrix3d p_pred = F * mixed_p[j] * F.transpose() + models_[j].Q;

    double innovation = normalize_angle(measured_yaw - x_pred[0]);
    double s = std::max(1e-9, p_pred(0, 0) + params_.r_yaw);
    Eigen::Vector3d k = p_pred.col(0) / s;

    Eigen::Vector3d x_upd = x_pred + k * innovation;
    x_upd[0] = normalize_angle(x_upd[0]);
    Eigen::Matrix3d p_upd = (Eigen::Matrix3d::Identity() - k * Eigen::RowVector3d(1, 0, 0)) * p_pred;

    models_[j].x = x_upd;
    models_[j].P = p_upd;

    const double gaussian = std::exp(-0.5 * innovation * innovation / s) / std::sqrt(2.0 * kPi * s);
    likelihood[j] = std::max(1e-12, gaussian);
  }

  double mu_sum = 0.0;
  for (size_t j = 0; j < 3; ++j) {
    mu_[j] = c[j] * likelihood[j];
    mu_sum += mu_[j];
  }
  mu_sum = std::max(mu_sum, 1e-12);
  for (auto & value : mu_) {
    value /= mu_sum;
  }

  std::array<Eigen::Vector3d, 3> states{models_[0].x, models_[1].x, models_[2].x};
  Eigen::Vector3d fused = blend_state(states, mu_);

  size_t dominant = 0;
  if (mu_[1] > mu_[dominant]) dominant = 1;
  if (mu_[2] > mu_[dominant]) dominant = 2;

  return {
    true,
    fused[0],
    fused[1],
    fused[2],
    {mu_[0], mu_[1], mu_[2]},
    static_cast<Mode>(dominant)};
}

}  // namespace auto_aim
