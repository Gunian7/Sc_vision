#include "tasks/auto_aim/hero_solver.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_aim {
namespace {

hero_coord::HeroCoordinateFrames load_coordinate_frames(const std::string& config_path)
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();

  const Eigen::Matrix3d R_gimbal2imubody =
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  const Eigen::Matrix3d R_camera2gimbal =
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  const Eigen::Vector3d t_camera2gimbal(t_camera2gimbal_data.data());

  Eigen::Vector3d joint_axis(0., 1., 0.);
  if (yaml["hero_joint_pitch_axis"].IsDefined()) {
    auto ax = yaml["hero_joint_pitch_axis"].as<std::vector<double>>();
    if (ax.size() >= 3) {
      joint_axis << ax[0], ax[1], ax[2];
    }
  }

  return hero_coord::HeroCoordinateFrames(
    R_gimbal2imubody, R_camera2gimbal, t_camera2gimbal, joint_axis);
}

} // namespace

HeroSolver::HeroSolver(const std::string& config_path)
: frames_(load_coordinate_frames(config_path))
{
}

void HeroSolver::set_board_orientation(const Eigen::Quaterniond& q)
{
  q_board_ = q.normalized();
  frames_.set_imu_quaternion(q_board_);
}

void HeroSolver::set_joint_pitch_rad(double rad)
{
  joint_pitch_rad_ = rad;
  frames_.set_joint_pitch_rad(rad);
}

Eigen::Isometry3d HeroSolver::lookup_transform(std::string_view target, std::string_view source) const
{
  return frames_.lookup_transform(target, source);
}

Eigen::Vector3d HeroSolver::transform_point(
  std::string_view target, std::string_view source, const Eigen::Vector3d& p_source) const
{
  return frames_.transform_point(target, source, p_source);
}

} // namespace auto_aim
