/**
 * Hero 坐标链实时自检：通过 HeroRosBoard 读 Autoaim，经 HeroSolver 输出各系姿态（与 Solver 相同 Z→Y→X 欧拉约定）。
 * 用法: hero_tf_test [config.yaml]  默认 configs/hero.yaml
 * Ctrl+C 退出。首包 Autoaim 到达前会在 imu_at 处阻塞。
 */
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "io/hero_ros_board/hero_ros_board.hpp"
#include "tasks/auto_aim/hero_solver.hpp"
#include "tools/exiter.hpp"
#include "tools/math_tools.hpp"

using HF = auto_aim::hero_coord::HeroCoordinateFrames;

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;

/** 与 Solver::solve 中 eulers(R, 2, 1, 0) 一致：内在 Z→Y→X，输出 [角_z, 角_y, 角_x]（弧度） */
Eigen::Vector3d euler_zyx_intrinsic_deg(const Eigen::Matrix3d& R)
{
  return tools::eulers(R, 2, 1, 0) * kRad2Deg;
}

void print_rpy_line(const char* label, const Eigen::Vector3d& zyx_deg)
{
  std::cout << label << " ZYX_intrinsic_deg [z,y,x]= " << zyx_deg.transpose() << "\n";
}

} // namespace

int main(int argc, char** argv)
{
  const std::string config = (argc >= 2) ? argv[1] : "configs/hero.yaml";

  rclcpp::init(argc, argv);

  io::HeroRosBoard hero_board(config);
  auto_aim::HeroSolver hero_solver(config);
  tools::Exiter exiter;

  std::cout << "=== hero_tf_test (live, HeroRosBoard + HeroSolver) ===\n";
  std::cout << "config: " << config << "\n";
  std::cout << "world 参考系 RPY 取 0（固定参考）；其余为相对 world 的旋转（gimbal/camera）。\n\n";

  while (!exiter.exit() && rclcpp::ok()) {
    const auto t = std::chrono::steady_clock::now();
    const Eigen::Quaterniond q = hero_board.imu_at(t);
    hero_solver.set_board_orientation(q);

    const io::HeroBoardParsed snap = hero_board.snapshot();
    if (snap.has_sample) {
      hero_solver.set_joint_pitch_rad(static_cast<double>(snap.vtx_pitch));
    }

    const Eigen::Matrix3d R_g2w = hero_solver.R_gimbal2world();
    const Eigen::Matrix3d R_c2g = hero_solver.R_camera2gimbal_effective();
    const Eigen::Isometry3d T_w_c = hero_solver.lookup_transform(HF::kWorld, HF::kCamera);
    const Eigen::Matrix3d R_c2w = T_w_c.linear();

    print_rpy_line("world (参考, 恒为 0)     ", Eigen::Vector3d::Zero());
    print_rpy_line("gimbal 相对 world         ", euler_zyx_intrinsic_deg(R_g2w));
    print_rpy_line("camera 相对 gimbal(有效)  ", euler_zyx_intrinsic_deg(R_c2g));
    print_rpy_line("camera 相对 world         ", euler_zyx_intrinsic_deg(R_c2w));

    std::cout << "cam_origin_world_m: " << T_w_c.translation().transpose()
              << " | v=" << hero_board.bullet_speed
              << " | board_raw_deg yaw=" << static_cast<double>(snap.high_gimbal_yaw) * kRad2Deg
              << " pitch=" << static_cast<double>(snap.pitch) * kRad2Deg
              << " vtx=" << static_cast<double>(snap.vtx_pitch) * kRad2Deg << "\n";
    std::cout << "---\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}
