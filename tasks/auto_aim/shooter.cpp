#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path)
: last_command_{false, false, 0, 0, 0, 0, 0, 0}, has_last_command_(false),
  yaw_jump_threshold_(35.0 / 57.3), yaw_jump_fire_cooldown_(0.0), has_yaw_jump_time_(false)
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();

  if (yaml["yaw_jump_threshold_deg"].IsDefined()) {
    yaw_jump_threshold_ = yaml["yaw_jump_threshold_deg"].as<double>() / 57.3;
  } else if (yaml["jump_yaw_threshold_deg"].IsDefined()) {
    yaw_jump_threshold_ = yaml["jump_yaw_threshold_deg"].as<double>() / 57.3;
  }

  if (yaml["yaw_jump_fire_cooldown"].IsDefined()) {
    yaw_jump_fire_cooldown_ = std::max(0.0, yaml["yaw_jump_fire_cooldown"].as<double>());
  } else if (yaml["jump_fire_cooldown"].IsDefined()) {
    yaw_jump_fire_cooldown_ = std::max(0.0, yaml["jump_fire_cooldown"].as<double>());
  }
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto now = std::chrono::steady_clock::now();
  if (!has_last_command_) {
    has_last_command_ = true;
    last_command_ = command;
    return false;
  }

  auto delta_cmd_yaw = std::abs(tools::limit_rad(command.yaw - last_command_.yaw));
  if (delta_cmd_yaw >= yaw_jump_threshold_) {
    has_yaw_jump_time_ = true;
    last_yaw_jump_time_ = now;
  }

  if (has_yaw_jump_time_ && yaw_jump_fire_cooldown_ > 0.0) {
    auto age = std::chrono::duration<double>(now - last_yaw_jump_time_).count();
    if (age >= 0.0 && age <= yaw_jump_fire_cooldown_) {
      last_command_ = command;
      return false;
    }
  }

  // 条件2：当云台yaw目标角速度>=17rad/s时不允许发弹
  if (std::abs(command.yaw_vel) >= 17.0) {
    last_command_ = command;
    return false;
  }

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim