#include "imm.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace auto_aim
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-12;
constexpr double kDefaultMuMin = 1e-4;
constexpr int kYawIndex = 6;
constexpr int kWIndex = 7;
constexpr int kAlphaIndex = 11;
}  // namespace

IMMFilter::IMMFilter() : initialized_(false)
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
  params_.mu_min = kDefaultMuMin;
  reset();
}

void IMMFilter::set_params(const Params & params)
{
  params_ = params;
  params_.r_yaw = std::max(1e-9, params_.r_yaw);
  params_.dt_min = std::max(1e-6, params_.dt_min);
  params_.dt_max = std::max(params_.dt_min, params_.dt_max);
  params_.alpha_decay_slow = std::clamp(params_.alpha_decay_slow, 0.0, 1.0);
  params_.alpha_decay_constant = std::clamp(params_.alpha_decay_constant, 0.0, 1.0);
  if (!(params_.mu_min > 0.0)) {
    params_.mu_min = kDefaultMuMin;
  }
  params_.mu_min = std::clamp(params_.mu_min, 0.0, 1.0 / static_cast<double>(kModelCount));
}

void IMMFilter::initialize(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0)
{
  initialized_ = true;
  fused_x_ = x0;
  fused_P_ = P0;
  if (fused_x_.size() > kYawIndex) {
    fused_x_[kYawIndex] = normalize_angle(fused_x_[kYawIndex]);
  }

  const double mu0 = 1.0 / static_cast<double>(kModelCount);
  c_bar_.fill(mu0);

  for (size_t i = 0; i < kModelCount; ++i) {
    auto & model = models_[i];
    model.x = x0;
    model.P = P0;
    model.mu = locked_model_index_ ? (i == *locked_model_index_ ? 1.0 : 0.0) : mu0;
    model.likelihood = 1.0;
    model.innovation = Eigen::VectorXd::Zero(4);
    model.innovation_cov = Eigen::MatrixXd::Zero(4, 4);
    if (model.x.size() > kYawIndex) {
      model.x[kYawIndex] = normalize_angle(model.x[kYawIndex]);
    }
  }
}

void IMMFilter::reset()
{
  initialized_ = false;
  locked_model_index_.reset();
  fused_x_.resize(0);
  fused_P_.resize(0, 0);
  c_bar_.fill(0.0);

  for (auto & model : models_) {
    model.x.resize(0);
    model.P.resize(0, 0);
    model.mu = 0.0;
    model.likelihood = 0.0;
    model.innovation.resize(0);
    model.innovation_cov.resize(0, 0);
  }
}

void IMMFilter::set_model_lock(std::optional<size_t> model_index)
{
  if (model_index && *model_index >= kModelCount) {
    model_index.reset();
  }

  locked_model_index_ = model_index;

  if (!initialized_) {
    return;
  }

  if (locked_model_index_) {
    for (size_t i = 0; i < kModelCount; ++i) {
      models_[i].mu = (i == *locked_model_index_) ? 1.0 : 0.0;
    }
  }
}

void IMMFilter::clear_model_lock()
{
  locked_model_index_.reset();

  if (!initialized_) {
    return;
  }

  const double mu0 = 1.0 / static_cast<double>(kModelCount);
  for (auto & model : models_) {
    model.mu = mu0;
  }
}

double IMMFilter::normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double IMMFilter::safe_det(const Eigen::MatrixXd & matrix)
{
  if (matrix.rows() == 0 || matrix.cols() == 0) {
    return 1.0;
  }

  const Eigen::MatrixXd sym = 0.5 * (matrix + matrix.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(sym);
  if (solver.info() != Eigen::Success) {
    return kEps;
  }

  double det = 1.0;
  for (int i = 0; i < solver.eigenvalues().size(); ++i) {
    det *= std::max(kEps, solver.eigenvalues()[i]);
  }
  return std::max(kEps, det);
}

Eigen::VectorXd IMMFilter::blend_state(
  const std::array<Eigen::VectorXd, kModelCount> & states,
  const std::array<double, kModelCount> & weights, int yaw_index)
{
  if (states[0].size() == 0) {
    return {};
  }

  const int dim = states[0].size();
  Eigen::VectorXd mixed = Eigen::VectorXd::Zero(dim);
  double c = 0.0;
  double s = 0.0;
  double weight_sum = 0.0;

  for (size_t i = 0; i < kModelCount; ++i) {
    if (states[i].size() != dim || weights[i] <= 0.0) {
      continue;
    }
    mixed += weights[i] * states[i];
    c += weights[i] * std::cos(states[i][yaw_index]);
    s += weights[i] * std::sin(states[i][yaw_index]);
    weight_sum += weights[i];
  }

  if (weight_sum <= kEps) {
    return states[0];
  }

  mixed[yaw_index] = std::atan2(s, c);
  return mixed;
}

Eigen::VectorXd IMMFilter::subtract_state(
  const Eigen::VectorXd & lhs, const Eigen::VectorXd & rhs, int yaw_index)
{
  Eigen::VectorXd delta = lhs - rhs;
  if (delta.size() > yaw_index) {
    delta[yaw_index] = normalize_angle(delta[yaw_index]);
  }
  return delta;
}

Eigen::Vector3d IMMFilter::model_q(size_t model_index) const
{
  switch (model_index) {
    case 0:
      return params_.q_slow;
    case 1:
      return params_.q_constant;
    default:
      return params_.q_variable;
  }
}

double IMMFilter::alpha_decay(size_t model_index) const
{
  switch (model_index) {
    case 0:
      return params_.alpha_decay_slow;
    case 1:
      return params_.alpha_decay_constant;
    default:
      return 1.0;
  }
}

SpinModel IMMFilter::spin_model(size_t model_index) const
{
  switch (model_index) {
    case 0:
      return SpinModel::slow;
    case 1:
      return SpinModel::constant;
    default:
      return SpinModel::variable;
  }
}

void IMMFilter::mix_states(int yaw_index)
{
  std::array<ModelState, kModelCount> prior = models_;

  for (size_t j = 0; j < kModelCount; ++j) {
    c_bar_[j] = 0.0;
    for (size_t i = 0; i < kModelCount; ++i) {
      c_bar_[j] += params_.transition(i, j) * std::max(0.0, prior[i].mu);
    }
    c_bar_[j] = std::max(kEps, c_bar_[j]);

    std::array<double, kModelCount> mix_weights{};
    double mix_weight_sum = 0.0;
    for (size_t i = 0; i < kModelCount; ++i) {
      mix_weights[i] = params_.transition(i, j) * std::max(0.0, prior[i].mu) / c_bar_[j];
      mix_weight_sum += mix_weights[i];
    }

    if (mix_weight_sum <= kEps) {
      mix_weights.fill(1.0 / static_cast<double>(kModelCount));
    }

    std::array<Eigen::VectorXd, kModelCount> states{
      prior[0].x, prior[1].x, prior[2].x};

    Eigen::VectorXd mixed_x = blend_state(states, mix_weights, yaw_index);
    Eigen::MatrixXd mixed_P = Eigen::MatrixXd::Zero(prior[0].P.rows(), prior[0].P.cols());

    for (size_t i = 0; i < kModelCount; ++i) {
      Eigen::VectorXd dx = subtract_state(prior[i].x, mixed_x, yaw_index);
      mixed_P += mix_weights[i] * (prior[i].P + dx * dx.transpose());
    }

    models_[j].x = mixed_x;
    models_[j].P = 0.5 * (mixed_P + mixed_P.transpose());
    models_[j].mu = c_bar_[j];
  }
}

void IMMFilter::fuse_output(int yaw_index)
{
  if (!initialized_ || models_[0].x.size() == 0) {
    return;
  }

  if (locked_model_index_) {
    const size_t j = *locked_model_index_;
    fused_x_ = models_[j].x;
    fused_P_ = models_[j].P;
    if (fused_x_.size() > yaw_index) {
      fused_x_[yaw_index] = normalize_angle(fused_x_[yaw_index]);
    }
    return;
  }

  std::array<double, kModelCount> weights{};
  double weight_sum = 0.0;
  for (size_t i = 0; i < kModelCount; ++i) {
    weights[i] = std::max(0.0, models_[i].mu);
    weight_sum += weights[i];
  }

  if (weight_sum <= kEps) {
    weights.fill(1.0 / static_cast<double>(kModelCount));
  } else {
    for (auto & weight : weights) {
      weight /= weight_sum;
    }
  }

  std::array<Eigen::VectorXd, kModelCount> states{
    models_[0].x, models_[1].x, models_[2].x};

  fused_x_ = blend_state(states, weights, yaw_index);
  fused_P_ = Eigen::MatrixXd::Zero(models_[0].P.rows(), models_[0].P.cols());

  for (size_t i = 0; i < kModelCount; ++i) {
    Eigen::VectorXd dx = subtract_state(models_[i].x, fused_x_, yaw_index);
    fused_P_ += weights[i] * (models_[i].P + dx * dx.transpose());
  }

  fused_P_ = 0.5 * (fused_P_ + fused_P_.transpose());
  if (fused_x_.size() > yaw_index) {
    fused_x_[yaw_index] = normalize_angle(fused_x_[yaw_index]);
  }
}

void IMMFilter::predict(const Target & target, double dt)
{
  if (!initialized_) {
    initialize(target.ekf_x(), target.ekf().P);
  }

  dt = std::clamp(dt, params_.dt_min, params_.dt_max);

  if (locked_model_index_) {
    const size_t j = *locked_model_index_;
    Eigen::MatrixXd F = target.state_transition_matrix(dt, spin_model(j));
    Eigen::VectorXd x_pred = target.predict_state(models_[j].x, dt, spin_model(j));

    if (spin_model(j) != SpinModel::variable) {
      const double decay = alpha_decay(j);
      F(kAlphaIndex, kAlphaIndex) = decay;
      x_pred[kAlphaIndex] = decay * models_[j].x[kAlphaIndex];
    }

    x_pred[kYawIndex] = normalize_angle(x_pred[kYawIndex]);
    Eigen::MatrixXd Q = target.process_noise_matrix(dt, model_q(j), spin_model(j));
    Eigen::MatrixXd P_pred = F * models_[j].P * F.transpose() + Q;

    models_[j].x = x_pred;
    models_[j].P = 0.5 * (P_pred + P_pred.transpose());
    models_[j].mu = 1.0;
    models_[j].likelihood = 1.0;

    for (size_t i = 0; i < kModelCount; ++i) {
      if (i == j) {
        continue;
      }
      models_[i].mu = 0.0;
    }

    fused_x_ = models_[j].x;
    fused_P_ = models_[j].P;
    if (fused_x_.size() > kYawIndex) {
      fused_x_[kYawIndex] = normalize_angle(fused_x_[kYawIndex]);
    }
    return;
  }

  mix_states(kYawIndex);

  for (size_t j = 0; j < kModelCount; ++j) {
    Eigen::MatrixXd F = target.state_transition_matrix(dt, spin_model(j));
    Eigen::VectorXd x_pred = target.predict_state(models_[j].x, dt, spin_model(j));

    if (spin_model(j) != SpinModel::variable) {
      const double decay = alpha_decay(j);
      F(kAlphaIndex, kAlphaIndex) = decay;
      x_pred[kAlphaIndex] = decay * models_[j].x[kAlphaIndex];
    }

    x_pred[kYawIndex] = normalize_angle(x_pred[kYawIndex]);
    Eigen::MatrixXd Q = target.process_noise_matrix(dt, model_q(j), spin_model(j));
    Eigen::MatrixXd P_pred = F * models_[j].P * F.transpose() + Q;

    models_[j].x = x_pred;
    models_[j].P = 0.5 * (P_pred + P_pred.transpose());
    models_[j].mu = c_bar_[j];
    models_[j].likelihood = 1.0;
  }

  fuse_output(kYawIndex);
}

bool IMMFilter::update(const Target & target, const Armor & armor, int armor_id)
{
  if (!initialized_) {
    return false;
  }

  const Eigen::Vector4d z = target.measurement_from_armor(armor);
  Eigen::MatrixXd R = target.measurement_noise_matrix(armor);
  if (R.rows() > 3 && R.cols() > 3) {
    R(3, 3) = std::max(R(3, 3), params_.r_yaw);
  }

  if (locked_model_index_) {
    const size_t j = *locked_model_index_;
    Eigen::MatrixXd H = target.h_jacobian(models_[j].x, armor_id);
    Eigen::Vector4d z_pred = target.predicted_measurement(models_[j].x, armor_id);
    Eigen::VectorXd innovation = target.measurement_subtract(z, z_pred);
    Eigen::MatrixXd S = H * models_[j].P * H.transpose() + R;
    S = 0.5 * (S + S.transpose());

    models_[j].innovation = innovation;
    models_[j].innovation_cov = S;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
      return false;
    }

    const Eigen::VectorXd solved_innovation = ldlt.solve(innovation);
    const double mahalanobis = innovation.transpose() * solved_innovation;
    const double det_s = safe_det(S);
    const double norm =
      std::pow(2.0 * kPi, static_cast<double>(z.size())) * std::max(det_s, kEps);
    double likelihood = std::exp(-0.5 * std::max(0.0, mahalanobis)) / std::sqrt(norm);

    if (!std::isfinite(likelihood) || likelihood < kEps) {
      likelihood = kEps;
    }

    const Eigen::MatrixXd identity =
      Eigen::MatrixXd::Identity(models_[j].x.size(), models_[j].x.size());
    const Eigen::MatrixXd PHt = models_[j].P * H.transpose();
    const Eigen::MatrixXd s_inv = ldlt.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
    const Eigen::MatrixXd K = PHt * s_inv;

    Eigen::VectorXd x_upd = models_[j].x + K * innovation;
    x_upd[kYawIndex] = normalize_angle(x_upd[kYawIndex]);

    const Eigen::MatrixXd joseph =
      (identity - K * H) * models_[j].P * (identity - K * H).transpose() +
      K * R * K.transpose();

    models_[j].x = x_upd;
    models_[j].P = 0.5 * (joseph + joseph.transpose());
    models_[j].mu = 1.0;
    models_[j].likelihood = likelihood;

    for (size_t i = 0; i < kModelCount; ++i) {
      if (i == j) {
        continue;
      }
      models_[i].mu = 0.0;
    }

    fused_x_ = models_[j].x;
    fused_P_ = models_[j].P;
    if (fused_x_.size() > kYawIndex) {
      fused_x_[kYawIndex] = normalize_angle(fused_x_[kYawIndex]);
    }
    return true;
  }

  std::array<double, kModelCount> posterior_mu{};
  double mu_sum = 0.0;

  for (size_t j = 0; j < kModelCount; ++j) {
    Eigen::MatrixXd H = target.h_jacobian(models_[j].x, armor_id);
    Eigen::Vector4d z_pred = target.predicted_measurement(models_[j].x, armor_id);
    Eigen::VectorXd innovation = target.measurement_subtract(z, z_pred);
    Eigen::MatrixXd S = H * models_[j].P * H.transpose() + R;
    S = 0.5 * (S + S.transpose());

    models_[j].innovation = innovation;
    models_[j].innovation_cov = S;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
      models_[j].likelihood = kEps;
      posterior_mu[j] = std::max(c_bar_[j], kEps) * models_[j].likelihood;
      mu_sum += posterior_mu[j];
      continue;
    }

    const Eigen::VectorXd solved_innovation = ldlt.solve(innovation);
    const double mahalanobis = innovation.transpose() * solved_innovation;
    const double det_s = safe_det(S);
    const double norm =
      std::pow(2.0 * kPi, static_cast<double>(z.size())) * std::max(det_s, kEps);
    double likelihood = std::exp(-0.5 * std::max(0.0, mahalanobis)) / std::sqrt(norm);

    if (!std::isfinite(likelihood) || likelihood < kEps) {
      likelihood = kEps;
    }

    const Eigen::MatrixXd identity =
      Eigen::MatrixXd::Identity(models_[j].x.size(), models_[j].x.size());
    const Eigen::MatrixXd PHt = models_[j].P * H.transpose();
    const Eigen::MatrixXd s_inv = ldlt.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
    const Eigen::MatrixXd K = PHt * s_inv;

    Eigen::VectorXd x_upd = models_[j].x + K * innovation;
    x_upd[kYawIndex] = normalize_angle(x_upd[kYawIndex]);

    const Eigen::MatrixXd joseph =
      (identity - K * H) * models_[j].P * (identity - K * H).transpose() +
      K * R * K.transpose();

    models_[j].x = x_upd;
    models_[j].P = 0.5 * (joseph + joseph.transpose());
    models_[j].likelihood = likelihood;

    posterior_mu[j] = std::max(c_bar_[j], kEps) * likelihood;
    mu_sum += posterior_mu[j];
  }

  if (mu_sum <= kEps) {
    mu_sum = 0.0;
    for (size_t j = 0; j < kModelCount; ++j) {
      posterior_mu[j] = std::max(c_bar_[j], kEps);
      mu_sum += posterior_mu[j];
    }
  }

  for (size_t j = 0; j < kModelCount; ++j) {
    posterior_mu[j] /= std::max(mu_sum, kEps);
  }

  if (params_.mu_min > 0.0) {
    for (auto & mu : posterior_mu) {
      mu = std::max(mu, params_.mu_min);
    }
    double renorm = 0.0;
    for (const auto mu : posterior_mu) {
      renorm += mu;
    }
    for (auto & mu : posterior_mu) {
      mu /= std::max(renorm, kEps);
    }
  }

  for (size_t j = 0; j < kModelCount; ++j) {
    models_[j].mu = posterior_mu[j];
  }

  fuse_output(kYawIndex);
  return true;
}

Eigen::VectorXd IMMFilter::state() const { return fused_x_; }

Eigen::MatrixXd IMMFilter::covariance() const { return fused_P_; }

std::array<double, IMMFilter::kModelCount> IMMFilter::getModelProbs() const
{
  std::array<double, kModelCount> probs{};
  for (size_t i = 0; i < kModelCount; ++i) {
    probs[i] = models_[i].mu;
  }
  return probs;
}

std::array<double, IMMFilter::kModelCount> IMMFilter::getModelAngularVelocitys() const
{
  std::array<double, kModelCount> ws{};
  for (size_t i = 0; i < kModelCount; ++i) {
    if (models_[i].x.size() > kWIndex) {
      ws[i] = models_[i].x[kWIndex];
    }
  }
  return ws;
}

std::array<double, IMMFilter::kModelCount> IMMFilter::getModelAngularAccelerations() const
{
  std::array<double, kModelCount> alphas{};
  for (size_t i = 0; i < kModelCount; ++i) {
    if (models_[i].x.size() > kAlphaIndex) {
      alphas[i] = models_[i].x[kAlphaIndex];
    }
  }
  return alphas;
}

}  // namespace auto_aim
