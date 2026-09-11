#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "imm.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  double jump_z_threshold_;
  double jump_yaw_threshold_rad_;
  int jump_confirm_count_;
  double jump_avg_alpha_;
  double jump_fire_cooldown_;
  double outpost_jump_fire_cooldown_;
  double jump_min_interval_;
  double process_noise_linear_normal_;
  double process_noise_angular_normal_;
  double process_noise_linear_outpost_;
  double process_noise_angular_outpost_;
  double measurement_noise_yaw_;
  double measurement_noise_pitch_;
  double target_match_gate_tracked_;
  double target_match_gate_init_;
  bool force_target_angular_velocity_;
  double forced_target_angular_velocity_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  SpinIMM spin_imm_;
  tools::Plotter plotter_;
  bool imm_enabled_;
  bool motion_state_enabled_;
  double motion_w_low_;
  double motion_dw_high_;
  bool imm_initialized_;
  double imm_last_w_;
  double imm_dw_lpf_;
  std::chrono::steady_clock::time_point imm_last_t_;

  // spin_state 滞回切换：抑制单帧野值和模型概率互抖
  SpinModel confirmed_spin_state_ = SpinModel::slow;
  SpinModel pending_spin_state_ = SpinModel::slow;
  int pending_spin_count_ = 0;
  int spin_confirm_frames_ = 4;
  double spin_switch_margin_ = 0.1;

  void state_machine(bool found);
  void update_motion_state(Target & target, std::chrono::steady_clock::time_point t);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
