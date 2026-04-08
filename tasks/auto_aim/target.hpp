#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool outpost_height_ready() const;
  int last_jump_dir() const;
  bool has_jump_time() const;
  std::chrono::steady_clock::time_point last_jump_time() const;
  void set_jump_params(double z_threshold, double yaw_threshold_rad, int confirm_count);
  void set_jump_avg_alpha(double alpha);
  void set_jump_fire_cooldown(double seconds);
  void set_jump_min_interval(double seconds);
  void set_process_noise(
    double linear_acc_normal, double angular_acc_normal, double linear_acc_outpost,
    double angular_acc_outpost);
  void set_measurement_noise(double yaw_noise, double pitch_noise);
  bool in_jump_fire_cooldown(std::chrono::steady_clock::time_point t) const;
  void set_angular_velocity(double angular_velocity);

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  bool height_init_done_;
  std::chrono::steady_clock::time_point height_init_start_;
  std::array<std::vector<double>, 3> height_samples_;
  std::array<double, 3> height_offsets_;
  int last_jump_dir_;
  bool has_jump_time_;
  std::chrono::steady_clock::time_point last_jump_time_;
  double jump_z_threshold_;
  double jump_yaw_threshold_rad_;
  int jump_confirm_count_;
  int jump_pending_dir_;
  int jump_pending_count_;
  double jump_avg_alpha_;
  std::array<double, 4> jump_avg_z_;
  std::array<bool, 4> jump_avg_inited_;
  double jump_fire_cooldown_;
  double jump_min_interval_;
  double process_noise_linear_normal_;
  double process_noise_angular_normal_;
  double process_noise_linear_outpost_;
  double process_noise_angular_outpost_;
  double measurement_noise_yaw_;
  double measurement_noise_pitch_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP