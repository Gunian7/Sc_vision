#ifndef AUTO_AIM__IMM_HPP
#define AUTO_AIM__IMM_HPP

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <optional>

namespace auto_aim
{

class IMMFilter
{
public:
  static constexpr int kModelCount = 3;
  static constexpr int kStateDim = 3;  // [yaw, v_yaw, alpha_yaw]

  struct ModelState
  {
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    double mu = 0.0;
    double likelihood = 0.0;
    double innovation = 0.0;
    double innovation_var = 0.0;
  };

  struct Params
  {
    Eigen::Matrix3d transition;
    double r_yaw;               // observation noise for yaw position (rad^2)
    double q_v_slow;
    double q_v_constant;
    double q_v_variable;
    double q_alpha_slow;
    double q_alpha_constant;
    double q_alpha_variable;
    double alpha_decay_slow;
    double alpha_decay_constant;
    double dt_min;
    double dt_max;
    double mu_min;
  };

  IMMFilter();

  void set_params(const Params & params);
  void initialize(double yaw, double v_yaw, double alpha_yaw, double P_yaw, double P_v, double P_alpha);
  void reset();

  void set_model_lock(std::optional<size_t> model_index);
  void clear_model_lock();
  bool model_locked() const { return locked_model_index_.has_value(); }
  std::optional<size_t> locked_model_index() const { return locked_model_index_; }

  bool initialized() const { return initialized_; }

  void predict(double dt);
  bool update(double observed_yaw, double r_yaw);

  double yaw() const;
  double v_yaw() const;
  double alpha_yaw() const;
  double yaw_cov() const;
  double v_yaw_cov() const;
  double alpha_yaw_cov() const;
  std::array<double, kModelCount> getModelProbs() const;
  std::array<double, kModelCount> getModelAngularVelocitys() const;
  std::array<double, kModelCount> getModelAngularAccelerations() const;

private:
  static constexpr int kYawIdx = 0;
  static constexpr int kVIdx = 1;
  static constexpr int kAlphaIdx = 2;

  bool initialized_;
  Params params_;
  std::optional<size_t> locked_model_index_;
  std::array<ModelState, kModelCount> models_;
  double fused_yaw_;
  double fused_v_yaw_;
  double fused_alpha_yaw_;
  double fused_P_yaw_;
  double fused_P_v_;
  double fused_P_alpha_;
  std::array<double, kModelCount> c_bar_;

  static double normalize_angle(double angle);
  static double safe_det(const Eigen::MatrixXd & matrix);

  double model_q_v(size_t model_index) const;
  double model_q_alpha(size_t model_index) const;
  double alpha_decay(size_t model_index) const;

  void mix_states();
  void fuse_output();
};

using SpinIMM = IMMFilter;

}  // namespace auto_aim

#endif  // AUTO_AIM__IMM_HPP
