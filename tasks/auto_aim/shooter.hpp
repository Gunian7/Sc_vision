#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <chrono>
#include <string>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

private:
  io::Command last_command_;
  bool has_last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  double yaw_jump_threshold_;
  double yaw_jump_fire_cooldown_;
  double outpost_tolerance_;
  double outpost_phase_window_;
  double outpost_yaw_vel_threshold_;
  bool has_yaw_jump_time_;
  std::chrono::steady_clock::time_point last_yaw_jump_time_;
  bool auto_fire_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP
