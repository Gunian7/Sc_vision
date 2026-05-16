#ifndef AUTO_AIM__IMM_HPP
#define AUTO_AIM__IMM_HPP

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <optional>

#include "armor.hpp"
#include "target.hpp"

namespace auto_aim
{

class IMMFilter
{
public:
  static constexpr int kModelCount = 3;

  struct ModelState
  {
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    double mu = 0.0;
    double likelihood = 0.0;
    Eigen::VectorXd innovation;
    Eigen::MatrixXd innovation_cov;
  };

  struct Params
  {
    Eigen::Matrix3d transition;
    double r_yaw;
    Eigen::Vector3d q_slow;
    Eigen::Vector3d q_constant;
    Eigen::Vector3d q_variable;
    double alpha_decay_slow;
    double alpha_decay_constant;
    double dt_min;
    double dt_max;
    double mu_min;
  };

  struct MatchResult
  {
    bool matched = false;
    int armor_index = -1;
    int armor_id = -1;
    double distance = 0.0;
  };

  IMMFilter();

  void set_params(const Params & params);
  void initialize(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0);
  void reset();

  void set_model_lock(std::optional<size_t> model_index);
  void clear_model_lock();
  bool model_locked() const { return locked_model_index_.has_value(); }
  std::optional<size_t> locked_model_index() const { return locked_model_index_; }

  bool initialized() const { return initialized_; }

  void predict(const Target & target, double dt);
  bool update(const Target & target, const Armor & armor, int armor_id);

  Eigen::VectorXd state() const;
  Eigen::MatrixXd covariance() const;
  std::array<double, kModelCount> getModelProbs() const;
  std::array<double, kModelCount> getModelAngularVelocitys() const;
  std::array<double, kModelCount> getModelAngularAccelerations() const;

private:
  bool initialized_;
  Params params_;
  std::optional<size_t> locked_model_index_;
  std::array<ModelState, kModelCount> models_;
  Eigen::VectorXd fused_x_;
  Eigen::MatrixXd fused_P_;
  std::array<double, kModelCount> c_bar_;

  static double normalize_angle(double angle);
  static double safe_det(const Eigen::MatrixXd & matrix);
  static Eigen::VectorXd blend_state(
    const std::array<Eigen::VectorXd, kModelCount> & states,
    const std::array<double, kModelCount> & weights, int yaw_index);
  static Eigen::VectorXd subtract_state(
    const Eigen::VectorXd & lhs, const Eigen::VectorXd & rhs, int yaw_index);

  Eigen::Vector3d model_q(size_t model_index) const;
  double alpha_decay(size_t model_index) const;
  SpinModel spin_model(size_t model_index) const;

  void mix_states(int yaw_index);
  void fuse_output(int yaw_index);
};

using SpinIMM = IMMFilter;

}  // namespace auto_aim

#endif  // AUTO_AIM__IMM_HPP
