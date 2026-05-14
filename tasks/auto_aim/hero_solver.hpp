#ifndef AUTO_AIM__HERO_SOLVER_HPP
#define AUTO_AIM__HERO_SOLVER_HPP

#include <Eigen/Geometry>
#include <string>
#include <string_view>

#include "tasks/auto_aim/hero_coordinate.hpp"

namespace auto_aim {

class Solver;

/**
 * 仅给 hero 节点使用：从下位机姿态维护与 standard / Solver 一致的 frame 链（固定 YAML R_camera2gimbal），
 * 提供 TF 式查询；并把 IMU 四元数同步到现有 Solver。
 */
class HeroSolver {
public:
  explicit HeroSolver(const std::string& config_path);

  /** 与 HeroRosBoard::imu_at / CBoard 同源的四元数（Z yaw × Y pitch）。 */
  void set_board_orientation(const Eigen::Quaterniond& q);

  /** 写入现有 Solver，等价于 standard.cpp 中 solver.set_R_gimbal2world(q)。 */
  void apply_to_solver(Solver& solver) const;

  Eigen::Quaterniond board_orientation() const { return q_board_; }

  Eigen::Matrix3d R_gimbal2world() const { return frames_.R_gimbal2world(); }
  Eigen::Matrix3d R_camera2gimbal_effective() const { return frames_.R_camera2gimbal_effective(); }

  /** p_target = lookup_transform(target, source) * p_source；frame: world | gimbal | camera */
  Eigen::Isometry3d lookup_transform(std::string_view target, std::string_view source) const;

  Eigen::Vector3d transform_point(
    std::string_view target, std::string_view source, const Eigen::Vector3d& p_source) const;

private:
  hero_coord::HeroCoordinateFrames frames_;
  Eigen::Quaterniond q_board_{Eigen::Quaterniond::Identity()};
};

} // namespace auto_aim

#endif
