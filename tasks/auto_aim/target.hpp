#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

enum class SpinModel : int
{
  slow = 0,
  constant = 1,
  variable = 2
};

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
  bool match_and_update(const std::vector<Armor> & armors);
  void apply_measurement_bookkeeping(const Armor & armor, int id);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  tools::ExtendedKalmanFilter & ekf();
  void set_filter_state(const Eigen::VectorXd & x, const Eigen::MatrixXd & P);
  std::vector<Eigen::Vector4d> armor_xyza_list() const;
  int match_armor_id(const Armor & armor, double * best_d2 = nullptr) const;
  Eigen::Vector4d measurement_from_armor(const Armor & armor) const;
  Eigen::MatrixXd measurement_noise_matrix(const Armor & armor) const;
  Eigen::Vector4d predicted_measurement(const Eigen::VectorXd & x, int id) const;
  Eigen::VectorXd measurement_subtract(const Eigen::VectorXd & a, const Eigen::VectorXd & b) const;
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd state_transition_matrix(double dt, SpinModel model) const;
  Eigen::MatrixXd process_noise_matrix(
    double dt, const Eigen::Vector3d & imm_q, SpinModel model) const;
  Eigen::VectorXd predict_state(const Eigen::VectorXd & x, double dt, SpinModel model) const;

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
  void set_match_gates(double tracked_gate, double init_gate);
  bool in_jump_fire_cooldown(std::chrono::steady_clock::time_point t) const;
  void set_angular_velocity(double angular_velocity);
  int armor_num() const { return armor_num_; }
  void set_spin_state(SpinModel state) { spin_state_ = state; }
  SpinModel spin_state() const { return spin_state_; }
  void set_linear_speed(double linear_speed) { linear_speed_ = linear_speed; }
  double linear_speed() const { return linear_speed_; }
  void set_imm_output(double yaw, double w, double alpha) { imm_yaw_ = yaw; imm_w_ = w; imm_alpha_ = alpha; }
  double imm_yaw() const { return imm_yaw_; }
  double imm_w() const { return imm_w_; }
  double imm_alpha() const { return imm_alpha_; }

  bool isinit = false;

  bool checkinit();

private:
  static constexpr int kStateDim = 12;
  static constexpr int kIdxX = 0;
  static constexpr int kIdxVx = 1;
  static constexpr int kIdxY = 2;
  static constexpr int kIdxVy = 3;
  static constexpr int kIdxZ = 4;
  static constexpr int kIdxVz = 5;
  static constexpr int kIdxYaw = 6;
  static constexpr int kIdxW = 7;
  static constexpr int kIdxR = 8;
  static constexpr int kIdxL = 9;
  static constexpr int kIdxH = 10;
  static constexpr int kIdxAlpha = 11;

  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  bool height_init_done_;
  std::chrono::steady_clock::time_point height_init_start_;
  std::array<std::vector<double>, 3> height_samples_;
  std::array<double, 3> height_offsets_;
  std::set<int> outpost_seen_ids_;
  bool outpost_all_ids_seen_;
  double outpost_height_min_gap_;
  double outpost_match_z_gate_;
  double outpost_match_z_penalty_scale_;
  double match_gate_tracked_;
  double match_gate_init_;
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
  SpinModel spin_state_;
  double linear_speed_;
  double imm_yaw_;
  double imm_w_;
  double imm_alpha_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  double robust_height_stat(const std::vector<double> & samples) const;
  void update_outpost_seen_ids(int id);
  void update_outpost_height_samples(const Armor & armor, int id);
  void update_switch_state(int id, const std::vector<Eigen::Vector4d> & xyza_list);
  void update_ypda(const Armor & armor, int id);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
