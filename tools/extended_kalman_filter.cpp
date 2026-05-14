#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  const Eigen::VectorXd x_prior = x;
  const Eigen::MatrixXd P_prior = P;
  const Eigen::VectorXd z_pred_prior = h(x_prior);
  const Eigen::VectorXd innovation = z_subtract(z, z_pred_prior);
  const Eigen::MatrixXd S = H * P_prior * H.transpose() + R;
  const Eigen::LDLT<Eigen::MatrixXd> innovation_ldlt(S);

  Eigen::MatrixXd K = P_prior * H.transpose() * S.inverse();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P_prior * (I - K * H).transpose() + K * R * K.transpose();

  x = x_add(x_prior, K * innovation);

  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;

  /// 卡方检验
  double nis = 0.0;
  if (innovation_ldlt.info() == Eigen::Success) {
    nis = innovation.transpose() * innovation_ldlt.solve(innovation);
  }

  double nees = 0.0;
  const Eigen::VectorXd dx = x - x_prior;
  const Eigen::LDLT<Eigen::MatrixXd> posterior_ldlt(P);
  if (posterior_ldlt.info() == Eigen::Success) {
    nees = dx.transpose() * posterior_ldlt.solve(dx);
  }

  // 卡方检验阈值：
  // NIS 使用量测维度 4、95% 分位数约 9.49
  // NEES 这里只保留统计，不参与 tracker 的重置判定
  constexpr double nis_threshold = 9.49;
  constexpr double nees_threshold = 9.49;

  if (std::isfinite(nis) && nis > nis_threshold) {
    nis_count_++;
    data["nis_fail"] = 1.0;
  }
  if (std::isfinite(nees) && nees > nees_threshold) {
    nees_count_++;
    data["nees_fail"] = 1.0;
  }
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back((std::isfinite(nis) && nis > nis_threshold) ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = innovation[0];
  data["residual_pitch"] = innovation[1];
  data["residual_distance"] = innovation[2];
  data["residual_angle"] = innovation[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools
