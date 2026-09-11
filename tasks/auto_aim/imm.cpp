#include "imm.hpp"

#include <Eigen/Eigenvalues>

#include "tools/logger.hpp"

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
}  // namespace

IMMFilter::IMMFilter() : initialized_(false)
{
  params_.transition << 0.93, 0.05, 0.02, 0.04, 0.93, 0.03, 0.03, 0.07, 0.90;
  params_.r_yaw = 2e-3;
  params_.q_v_slow = 8e-2;
  params_.q_v_constant = 2e-2;
  params_.q_v_variable = 2e-1;
  params_.q_alpha_slow = 1e-1;
  params_.q_alpha_constant = 5e-2;
  params_.q_alpha_variable = 8e-1;
  params_.alpha_decay_slow = 0.2;
  params_.alpha_decay_constant = 0.5;
  params_.dt_min = 1e-3;
  params_.dt_max = 0.2;
  params_.mu_min = kDefaultMuMin;
  params_.nis_gate = 9.0;
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
  params_.nis_gate = std::max(1.0, params_.nis_gate);
}

void IMMFilter::initialize(double yaw, double v_yaw, double alpha_yaw, double P_yaw, double P_v, double P_alpha)
{
  initialized_ = true;
  fused_yaw_ = yaw;
  fused_v_yaw_ = v_yaw;
  fused_alpha_yaw_ = alpha_yaw;
  fused_P_yaw_ = std::max(kEps, P_yaw);
  fused_P_v_ = std::max(kEps, P_v);
  fused_P_alpha_ = std::max(kEps, P_alpha);

  const double mu0 = 1.0 / static_cast<double>(kModelCount);
  c_bar_.fill(mu0);

  for (size_t i = 0; i < kModelCount; ++i) {
    auto & model = models_[i];
    model.x = Eigen::Vector3d(yaw, v_yaw, alpha_yaw);
    model.P = Eigen::Matrix3d::Identity();
    model.P(0, 0) = fused_P_yaw_;
    model.P(1, 1) = fused_P_v_;
    model.P(2, 2) = fused_P_alpha_;
    model.mu = locked_model_index_ ? (i == *locked_model_index_ ? 1.0 : 0.0) : mu0;
    model.likelihood = 1.0;
    model.innovation = 0.0;
    model.innovation_var = 0.0;
  }
}

void IMMFilter::reset()
{
  initialized_ = false;
  locked_model_index_.reset();
  fused_yaw_ = 0.0;
  fused_v_yaw_ = 0.0;
  fused_alpha_yaw_ = 0.0;
  fused_P_yaw_ = 0.0;
  fused_P_v_ = 0.0;
  fused_P_alpha_ = 0.0;
  c_bar_.fill(0.0);

  for (auto & model : models_) {
    model.x.resize(0);
    model.P.resize(0, 0);
    model.mu = 0.0;
    model.likelihood = 0.0;
    model.innovation = 0.0;
    model.innovation_var = 0.0;
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

double IMMFilter::model_q_v(size_t model_index) const
{
  switch (model_index) {
    case 0:
      return params_.q_v_slow;
    case 1:
      return params_.q_v_constant;
    default:
      return params_.q_v_variable;
  }
}

double IMMFilter::model_q_alpha(size_t model_index) const
{
  switch (model_index) {
    case 0:
      return params_.q_alpha_slow;
    case 1:
      return params_.q_alpha_constant;
    default:
      return params_.q_alpha_variable;
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

void IMMFilter::mix_states()
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

    // Mix 3D states [yaw, v_yaw, alpha_yaw]
    // yaw is circular: unwrap each model's yaw onto the branch of the
    // highest-weight model before averaging, otherwise mixing across the
    // +/-pi boundary yields a value up to pi away from the true angle
    size_t ref = 0;
    for (size_t i = 1; i < kModelCount; ++i) {
      if (mix_weights[i] > mix_weights[ref]) ref = i;
    }
    const double ref_yaw = prior[ref].x[kYawIdx];

    double mixed_yaw = 0.0;
    double mixed_v = 0.0;
    double mixed_alpha = 0.0;
    for (size_t i = 0; i < kModelCount; ++i) {
      const double yaw_i = ref_yaw + normalize_angle(prior[i].x[kYawIdx] - ref_yaw);
      mixed_yaw += mix_weights[i] * yaw_i;
      mixed_v += mix_weights[i] * prior[i].x[kVIdx];
      mixed_alpha += mix_weights[i] * prior[i].x[kAlphaIdx];
    }
    mixed_yaw = normalize_angle(mixed_yaw);

    Eigen::Matrix3d mixed_P = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < kModelCount; ++i) {
      const double yaw_i = ref_yaw + normalize_angle(prior[i].x[kYawIdx] - ref_yaw);
      Eigen::Vector3d dx(yaw_i - mixed_yaw, prior[i].x[kVIdx] - mixed_v,
                         prior[i].x[kAlphaIdx] - mixed_alpha);
      mixed_P += mix_weights[i] * (prior[i].P + dx * dx.transpose());
    }

    models_[j].x = Eigen::Vector3d(mixed_yaw, mixed_v, mixed_alpha);
    models_[j].P = 0.5 * (mixed_P + mixed_P.transpose());
    models_[j].mu = c_bar_[j];
  }
}

void IMMFilter::fuse_output()
{
  if (!initialized_ || models_[0].x.size() == 0) {
    return;
  }

  if (locked_model_index_) {
    const size_t j = *locked_model_index_;
    fused_yaw_ = models_[j].x[kYawIdx];
    fused_v_yaw_ = models_[j].x[kVIdx];
    fused_alpha_yaw_ = models_[j].x[kAlphaIdx];
    fused_P_yaw_ = models_[j].P(kYawIdx, kYawIdx);
    fused_P_v_ = models_[j].P(kVIdx, kVIdx);
    fused_P_alpha_ = models_[j].P(kAlphaIdx, kAlphaIdx);
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

  // Fuse yaw (using circular mean)
  double sin_sum = 0.0, cos_sum = 0.0;
  for (size_t i = 0; i < kModelCount; ++i) {
    sin_sum += weights[i] * std::sin(models_[i].x[kYawIdx]);
    cos_sum += weights[i] * std::cos(models_[i].x[kYawIdx]);
  }
  fused_yaw_ = std::atan2(sin_sum, cos_sum);
  // Normalize fused yaw
  fused_yaw_ = normalize_angle(fused_yaw_);

  // Fuse v_yaw
  fused_v_yaw_ = 0.0;
  for (size_t i = 0; i < kModelCount; ++i) {
    fused_v_yaw_ += weights[i] * models_[i].x[kVIdx];
  }

  // Fuse alpha_yaw
  fused_alpha_yaw_ = 0.0;
  for (size_t i = 0; i < kModelCount; ++i) {
    fused_alpha_yaw_ += weights[i] * models_[i].x[kAlphaIdx];
  }

  // Fuse covariance for yaw
  fused_P_yaw_ = 0.0;
  for (size_t i = 0; i < kModelCount; ++i) {
    double dyaw = normalize_angle(models_[i].x[kYawIdx] - fused_yaw_);
    fused_P_yaw_ += weights[i] * (models_[i].P(kYawIdx, kYawIdx) + dyaw * dyaw);
  }

  // Fuse covariance for v_yaw
  fused_P_v_ = 0.0;
  for (size_t i = 0; i < kModelCount; ++i) {
    double dv = models_[i].x[kVIdx] - fused_v_yaw_;
    fused_P_v_ += weights[i] * (models_[i].P(kVIdx, kVIdx) + dv * dv);
  }

  // Fuse covariance for alpha_yaw
  fused_P_alpha_ = 0.0;
  for (size_t i = 0; i < kModelCount; ++i) {
    double da = models_[i].x[kAlphaIdx] - fused_alpha_yaw_;
    fused_P_alpha_ += weights[i] * (models_[i].P(kAlphaIdx, kAlphaIdx) + da * da);
  }

  // PSD watchdog: model covariance must stay positive definite
  for (size_t i = 0; i < kModelCount; ++i) {
    if (models_[i].x.size() == 0) {
      continue;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
      0.5 * (models_[i].P + models_[i].P.transpose()));
    if (solver.info() != Eigen::Success) {
      continue;
    }
    const double min_eigenvalue = solver.eigenvalues().minCoeff();
    if (min_eigenvalue < -1e-9) {
      static int psd_warn_count = 0;
      if (++psd_warn_count % 100 == 1) {
        tools::logger()->warn(
          "[IMM] model {} covariance lost PSD, min eigenvalue {:.3e}", i, min_eigenvalue);
      }
    }
  }
}

void IMMFilter::predict(double dt)
{
  if (!initialized_) {
    return;
  }

  dt = std::clamp(dt, params_.dt_min, params_.dt_max);

  if (locked_model_index_) {
    const size_t j = *locked_model_index_;
    auto & model = models_[j];

    const double decay = alpha_decay(j);
    const double yaw = model.x[kYawIdx];
    const double v = model.x[kVIdx];
    const double alpha = model.x[kAlphaIdx] * decay;

    // CA model: yaw += v*dt + 0.5*alpha*dt^2, v += alpha*dt, alpha *= decay
    const double dt2 = dt * dt;
    double yaw_pred = yaw + v * dt + 0.5 * alpha * dt2;
    double v_pred = v + alpha * dt;
    double alpha_pred = alpha * decay;

    // 3x3 state transition matrix
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(kYawIdx, kVIdx) = dt;
    F(kYawIdx, kAlphaIdx) = 0.5 * dt2;
    F(kVIdx, kAlphaIdx) = dt;
    F(kAlphaIdx, kAlphaIdx) = decay;

    // 3x3 process noise (diagonal)
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    Q(kYawIdx, kYawIdx) = std::max(0.0, model_q_v(j)) * dt;
    Q(kVIdx, kVIdx) = std::max(0.0, model_q_v(j));
    Q(kAlphaIdx, kAlphaIdx) = std::max(0.0, model_q_alpha(j));

    Eigen::Matrix3d P_pred = F * model.P * F.transpose() + Q;

    model.x = Eigen::Vector3d(yaw_pred, v_pred, alpha_pred);
    model.P = 0.5 * (P_pred + P_pred.transpose());
    model.mu = 1.0;
    model.likelihood = 1.0;

    for (size_t i = 0; i < kModelCount; ++i) {
      if (i == j) {
        continue;
      }
      models_[i].mu = 0.0;
    }

    fused_yaw_ = model.x[kYawIdx];
    fused_v_yaw_ = model.x[kVIdx];
    fused_alpha_yaw_ = model.x[kAlphaIdx];
    fused_P_yaw_ = model.P(kYawIdx, kYawIdx);
    fused_P_v_ = model.P(kVIdx, kVIdx);
    fused_P_alpha_ = model.P(kAlphaIdx, kAlphaIdx);
    return;
  }

  mix_states();

  for (size_t j = 0; j < kModelCount; ++j) {
    auto & model = models_[j];
    const double decay = alpha_decay(j);
    const double yaw = model.x[kYawIdx];
    const double v = model.x[kVIdx];
    const double alpha = model.x[kAlphaIdx] * decay;

    // CA model for all 3 models
    const double dt2 = dt * dt;
    double yaw_pred = yaw + v * dt + 0.5 * alpha * dt2;
    double v_pred = v + alpha * dt;
    double alpha_pred = alpha * decay;

    // 3x3 state transition matrix
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(kYawIdx, kVIdx) = dt;
    F(kYawIdx, kAlphaIdx) = 0.5 * dt2;
    F(kVIdx, kAlphaIdx) = dt;
    F(kAlphaIdx, kAlphaIdx) = decay;

    // 3x3 process noise (diagonal)
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    Q(kYawIdx, kYawIdx) = std::max(0.0, model_q_v(j)) * dt;
    Q(kVIdx, kVIdx) = std::max(0.0, model_q_v(j));
    Q(kAlphaIdx, kAlphaIdx) = std::max(0.0, model_q_alpha(j));

    Eigen::Matrix3d P_pred = F * model.P * F.transpose() + Q;

    model.x = Eigen::Vector3d(yaw_pred, v_pred, alpha_pred);
    model.P = 0.5 * (P_pred + P_pred.transpose());
    model.mu = c_bar_[j];
    model.likelihood = 1.0;
  }

  fuse_output();
}

bool IMMFilter::update(double observed_yaw, double r_yaw)
{
  if (!initialized_) {
    return false;
  }

  const double r = std::max(kEps, r_yaw);

  if (locked_model_index_) {
    const size_t j = *locked_model_index_;
    auto & model = models_[j];

    // H = [1, 0, 0] - observe yaw position
    double innovation = normalize_angle(observed_yaw - model.x[kYawIdx]);
    const double S = std::max(model.P(kYawIdx, kYawIdx) + r, 1e-6);

    model.innovation = innovation;
    model.innovation_var = S;

    // χ² outlier gate: an implausible observation must not correct the model
    if (innovation * innovation / S > params_.nis_gate) {
      model.likelihood = 1.0;
      return true;  // fused 状态保持 predict 的预测值
    }

    // Kalman gain for 3x1 H = [1, 0, 0]
    const double K_yaw = model.P(kYawIdx, kYawIdx) / S;
    const double K_v = model.P(kVIdx, kYawIdx) / S;
    const double K_alpha = model.P(kAlphaIdx, kYawIdx) / S;

    // Update state
    double yaw_upd = normalize_angle(model.x[kYawIdx] + K_yaw * innovation);
    double v_upd = model.x[kVIdx] + K_v * innovation;
    double alpha_upd = model.x[kAlphaIdx] + K_alpha * innovation;

    // Joseph form covariance update: P = (I-KH) P (I-KH)^T + K r K^T
    // H = [1, 0, 0], so I-KH is the identity with column 0 subtracted by K.
    // 用矩阵表达式而不是手写展开，避免逐项推导时转置方向出错
    const Eigen::Matrix<double, 3, 1> K_vec(K_yaw, K_v, K_alpha);
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    A.col(0) -= K_vec;
    const Eigen::Matrix3d P_upd =
      A * model.P * A.transpose() + K_vec * r * K_vec.transpose();

    model.x = Eigen::Vector3d(yaw_upd, v_upd, alpha_upd);
    model.P = 0.5 * (P_upd + P_upd.transpose());

    // Likelihood
    const double mahalanobis = innovation * innovation / std::max(kEps, S);
    const double norm = std::sqrt(2.0 * kPi * std::max(kEps, S));
    double likelihood = std::exp(-0.5 * mahalanobis) / norm;
    if (!std::isfinite(likelihood) || likelihood < kEps) {
      likelihood = kEps;
    }

    model.mu = 1.0;
    model.likelihood = likelihood;

    for (size_t i = 0; i < kModelCount; ++i) {
      if (i == j) {
        continue;
      }
      models_[i].mu = 0.0;
    }

    fused_yaw_ = model.x[kYawIdx];
    fused_v_yaw_ = model.x[kVIdx];
    fused_alpha_yaw_ = model.x[kAlphaIdx];
    fused_P_yaw_ = model.P(kYawIdx, kYawIdx);
    fused_P_v_ = model.P(kVIdx, kVIdx);
    fused_P_alpha_ = model.P(kAlphaIdx, kAlphaIdx);
    return true;
  }

  std::array<double, kModelCount> posterior_mu{};
  double mu_sum = 0.0;

  for (size_t j = 0; j < kModelCount; ++j) {
    auto & model = models_[j];

    // H = [1, 0, 0] - observe yaw position
    double innovation = normalize_angle(observed_yaw - model.x[kYawIdx]);
    const double S = std::max(model.P(kYawIdx, kYawIdx) + r, 1e-6);

    model.innovation = innovation;
    model.innovation_var = S;

    const double mahalanobis = innovation * innovation / S;
    double likelihood = kEps;

    // χ² outlier gate: an implausible observation must not correct the model,
    // 其似然被压到下限，模型概率自然回落而不会被野值拉满
    if (mahalanobis <= params_.nis_gate) {
      // Kalman gain for 3x1 H = [1, 0, 0]
      const double K_yaw = model.P(kYawIdx, kYawIdx) / S;
      const double K_v = model.P(kVIdx, kYawIdx) / S;
      const double K_alpha = model.P(kAlphaIdx, kYawIdx) / S;

      // Update state
      double yaw_upd = normalize_angle(model.x[kYawIdx] + K_yaw * innovation);
      double v_upd = model.x[kVIdx] + K_v * innovation;
      double alpha_upd = model.x[kAlphaIdx] + K_alpha * innovation;

      // Joseph form covariance update: P = (I-KH) P (I-KH)^T + K r K^T
      // H = [1, 0, 0], so I-KH is the identity with column 0 subtracted by K.
      const Eigen::Matrix<double, 3, 1> K_vec(K_yaw, K_v, K_alpha);
      Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
      A.col(0) -= K_vec;
      const Eigen::Matrix3d P_upd =
        A * model.P * A.transpose() + K_vec * r * K_vec.transpose();

      model.x = Eigen::Vector3d(yaw_upd, v_upd, alpha_upd);
      model.P = 0.5 * (P_upd + P_upd.transpose());

      const double norm = std::sqrt(2.0 * kPi * S);
      likelihood = std::exp(-0.5 * mahalanobis) / norm;
      if (!std::isfinite(likelihood) || likelihood < kEps) {
        likelihood = kEps;
      }
    }

    model.likelihood = likelihood;
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

  fuse_output();
  return true;
}

double IMMFilter::yaw() const { return fused_yaw_; }

double IMMFilter::v_yaw() const { return fused_v_yaw_; }

double IMMFilter::alpha_yaw() const { return fused_alpha_yaw_; }

double IMMFilter::yaw_cov() const { return fused_P_yaw_; }

double IMMFilter::v_yaw_cov() const { return fused_P_v_; }

double IMMFilter::alpha_yaw_cov() const { return fused_P_alpha_; }

double IMMFilter::innovation_var() const
{
  if (!initialized_) {
    return 0.0;
  }
  size_t best = 0;
  for (size_t i = 1; i < kModelCount; ++i) {
    if (models_[i].mu > models_[best].mu) best = i;
  }
  return models_[best].innovation_var;
}

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
    if (models_[i].x.size() > kVIdx) {
      ws[i] = models_[i].x[kVIdx];
    }
  }
  return ws;
}

std::array<double, IMMFilter::kModelCount> IMMFilter::getModelAngularAccelerations() const
{
  std::array<double, kModelCount> alphas{};
  for (size_t i = 0; i < kModelCount; ++i) {
    if (models_[i].x.size() > kAlphaIdx) {
      alphas[i] = models_[i].x[kAlphaIdx];
    }
  }
  return alphas;
}

}  // namespace auto_aim
