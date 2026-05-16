#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "auto_aim_fsm.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double comming_angle_high_;
  double leaving_angle_high_;
  double speed_angle_;
  double comming_end_angle_;
  double leaving_end_angle_;
  double speed_angle_max_;
  double outpost_comming_angle_;
  double outpost_leaving_angle_;
  int lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double outpost_delay_time_;
  double decision_speed_;
  bool use_center_aim_when_high_speed_;
  double pre_aim_max_delta_angle_;
  bool fsm_enable_;
  AutoAimFsmController fsm_controller_;
  AutoAimFsm fsm_state_;

  AimPoint choose_aim_point(const Target & target, double fly_time = 0.0);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
