#ifndef AUTO_AIM__IMM_HPP
#define AUTO_AIM__IMM_HPP

#include <Eigen/Dense>
#include <array>

namespace auto_aim
{

class SpinIMM
{
public:
  enum class Mode : int
  {
    slow_spin = 0,
    constant_spin = 1,
    variable_spin = 2
  };

  struct Output
  {
    bool initialized;
    double yaw;
    double w;
    double alpha;
    std::array<double, 3> mode_probability;
    Mode dominant_mode;
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
  };

  SpinIMM();
  void set_params(const Params & params);

  void reset(double yaw, double w = 0.0, double alpha = 0.0);
  Output update(double measured_yaw, double dt);

private:
  struct Model
  {
    Eigen::Vector3d x;
    Eigen::Matrix3d P;
    Eigen::Matrix3d Q;
  };

  bool initialized_;
  Params params_;
  std::array<Model, 3> models_;
  std::array<double, 3> mu_;

  static double normalize_angle(double angle);
  static Eigen::Vector3d blend_state(
    const std::array<Eigen::Vector3d, 3> & states, const std::array<double, 3> & weights);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__IMM_HPP
