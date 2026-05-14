/**
 * Hero 坐标链实时自检：通过 HeroRosBoard 读 Autoaim，经 HeroSolver 输出各系姿态（与 Solver 相同 Z→Y→X 欧拉约定）。
 * 用法: hero_tf_test [config.yaml] [--no_viz]
 *   默认 configs/hero.yaml；不加 --no_viz 时打开两幅示意图（俯视图 yaw / 侧视图 pitch）校对轴向约定。
 * Ctrl+C 退出。首包 Autoaim 到达前会在 imu_at 处阻塞。
 */
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include "io/hero_config_path.hpp"
#include "io/hero_ros_board/hero_ros_board.hpp"
#include "tasks/auto_aim/hero_solver.hpp"
#include "tools/exiter.hpp"
#include "tools/math_tools.hpp"

using HF = auto_aim::hero_coord::HeroCoordinateFrames;

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;

constexpr int kCanvas = 440;
constexpr double kAxisLen = 0.95;  // 单位轴在图上的相对长度（归一化后乘 scale）

/** 与 Solver::solve 中 eulers(R, 2, 1, 0) 一致：内在 Z→Y→X，输出 [角_z, 角_y, 角_x]（弧度） */
Eigen::Vector3d euler_zyx_intrinsic_deg(const Eigen::Matrix3d & R)
{
  return tools::eulers(R, 2, 1, 0) * kRad2Deg;
}

void print_rpy_line(const char * label, const Eigen::Vector3d & zyx_deg)
{
  std::cout << label << " ZYX_intrinsic_deg [z,y,x]= " << zyx_deg.transpose() << "\n";
}

void draw_arrow_bgr(
  cv::Mat & img, cv::Point2d o, cv::Point2d dir, const cv::Scalar & bgr, int thickness = 2)
{
  const double n = std::hypot(dir.x, dir.y);
  if (n < 1e-9) {
    return;
  }
  const cv::Point2d e = o + dir;
  cv::arrowedLine(img, o, e, bgr, thickness, cv::LINE_AA, 0, 0.12);
}

/** 俯视图：world 水平面 XY；校对 yaw 时看云台 X/Y 在水平面投影与 world 轴关系 */
cv::Mat make_yaw_xy_panel(const Eigen::Matrix3d & R_g2w, const Eigen::Vector3d & gimbal_zyx_deg)
{
  cv::Mat img(kCanvas, kCanvas, CV_8UC3, cv::Scalar(255, 255, 255));
  const cv::Point2d c(kCanvas * 0.5, kCanvas * 0.5);
  const double s = kCanvas * 0.38;

  auto to_screen = [c, s](double wx, double wy) {
    return cv::Point2d(c.x + s * wx, c.y - s * wy);
  };

  const cv::Scalar col_wx(0, 0, 200);
  const cv::Scalar col_wy(0, 160, 0);
  const cv::Scalar col_gx(80, 80, 255);
  const cv::Scalar col_gy(60, 200, 60);
  const cv::Scalar col_gz(255, 120, 0);

  // world 参考：+X、+Y（细线）
  cv::line(img, c, to_screen(kAxisLen, 0), col_wx, 1, cv::LINE_AA);
  cv::line(img, c, to_screen(0, kAxisLen), col_wy, 1, cv::LINE_AA);

  // 云台轴在 world 下的表达：列向量为 gimbal 基矢在 world 的分量
  const Eigen::Vector3d gx = R_g2w.col(0);
  const Eigen::Vector3d gy = R_g2w.col(1);
  const Eigen::Vector3d gz = R_g2w.col(2);

  draw_arrow_bgr(img, c, to_screen(kAxisLen * gx.x(), kAxisLen * gx.y()) - c, col_gx, 2);
  draw_arrow_bgr(img, c, to_screen(kAxisLen * gy.x(), kAxisLen * gy.y()) - c, col_gy, 2);
  draw_arrow_bgr(img, c, to_screen(0.55 * gz.x(), 0.55 * gz.y()) - c, col_gz, 1);

  cv::putText(
    img, "yaw: world XY (top +Z)", {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {40, 40, 40}, 1, cv::LINE_AA);
  cv::putText(
    img, "thin Wx,Wy  solid Gx,Gy  thin Gz_xy", {12, 52}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {60, 60, 60}, 1,
    cv::LINE_AA);
  std::string line = cv::format(
    "gimbal ZYX_deg [z,y,x]: %.1f %.1f %.1f", gimbal_zyx_deg[0], gimbal_zyx_deg[1], gimbal_zyx_deg[2]);
  cv::putText(img, line, {12, kCanvas - 18}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 0, 0}, 1, cv::LINE_AA);
  return img;
}

/** 侧视图：world XZ（从 +Y 看向原点）；校对 pitch 时看云台 X/Z 在立面投影 */
cv::Mat make_pitch_xz_panel(const Eigen::Matrix3d & R_g2w, const Eigen::Vector3d & gimbal_zyx_deg)
{
  cv::Mat img(kCanvas, kCanvas, CV_8UC3, cv::Scalar(255, 255, 255));
  const cv::Point2d c(kCanvas * 0.5, kCanvas * 0.52);
  const double s = kCanvas * 0.38;

  auto to_screen = [c, s](double wx, double wz) {
    return cv::Point2d(c.x + s * wx, c.y - s * wz);
  };

  const cv::Scalar col_wx(0, 0, 200);
  const cv::Scalar col_wz(200, 0, 0);
  const cv::Scalar col_gx(80, 80, 255);
  const cv::Scalar col_gz(200, 100, 0);

  cv::line(img, c, to_screen(kAxisLen, 0), col_wx, 1, cv::LINE_AA);
  cv::line(img, c, to_screen(0, kAxisLen), col_wz, 1, cv::LINE_AA);

  const Eigen::Vector3d gx = R_g2w.col(0);
  const Eigen::Vector3d gz = R_g2w.col(2);

  draw_arrow_bgr(img, c, to_screen(kAxisLen * gx.x(), kAxisLen * gx.z()) - c, col_gx, 2);
  draw_arrow_bgr(img, c, to_screen(kAxisLen * gz.x(), kAxisLen * gz.z()) - c, col_gz, 2);

  cv::putText(
    img, "pitch: world XZ (from +Y)", {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {40, 40, 40}, 1, cv::LINE_AA);
  cv::putText(
    img, "thin Wx,Wz  solid Gx,Gz", {12, 52}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {60, 60, 60}, 1, cv::LINE_AA);
  std::string line = cv::format(
    "gimbal ZYX_deg [z,y,x]: %.1f %.1f %.1f", gimbal_zyx_deg[0], gimbal_zyx_deg[1], gimbal_zyx_deg[2]);
  cv::putText(img, line, {12, kCanvas - 18}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 0, 0}, 1, cv::LINE_AA);
  return img;
}

}  // namespace

int main(int argc, char ** argv)
{
  bool no_viz = false;
  std::string config_arg = "configs/hero.yaml";
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--no_viz") {
      no_viz = true;
    } else if (!a.empty() && a[0] != '-') {
      config_arg = a;
    }
  }

  const std::string config = io::resolve_config_path_next_to_build(config_arg, argv[0]);

  rclcpp::init(argc, argv);

  io::HeroRosBoard hero_board(config);
  auto_aim::HeroSolver hero_solver(config);
  tools::Exiter exiter;

  std::cout << "=== hero_tf_test (live, HeroRosBoard + HeroSolver) ===\n";
  std::cout << "config: " << config << "\n";
  std::cout << "world 参考系 RPY 取 0（固定参考）；其余为相对 world 的旋转（gimbal/camera）。\n";
  if (!no_viz) {
    std::cout << "viz: 左/窗1 俯视图(world XY) 校 yaw；右/窗2 侧视图(world XZ) 校 pitch。（--no_viz 关闭）\n";
  }
  std::cout << "\n";

  if (!no_viz) {
    cv::namedWindow("hero_tf yaw (world XY)", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("hero_tf pitch (world XZ)", cv::WINDOW_AUTOSIZE);
  }

  while (!exiter.exit() && rclcpp::ok()) {
    const auto t = std::chrono::steady_clock::now();
    const Eigen::Quaterniond q = hero_board.imu_at(t);
    hero_solver.set_board_orientation(q);

    const io::HeroBoardParsed snap = hero_board.snapshot();

    const Eigen::Matrix3d R_g2w = hero_solver.R_gimbal2world();
    const Eigen::Matrix3d R_c2g = hero_solver.R_camera2gimbal_effective();
    const Eigen::Isometry3d T_w_c = hero_solver.lookup_transform(HF::kWorld, HF::kCamera);
    const Eigen::Matrix3d R_c2w = T_w_c.linear();

    print_rpy_line("world (参考, 恒为 0)     ", Eigen::Vector3d::Zero());
    print_rpy_line("gimbal 相对 world         ", euler_zyx_intrinsic_deg(R_g2w));
    print_rpy_line("camera 相对 gimbal(YAML)  ", euler_zyx_intrinsic_deg(R_c2g));
    print_rpy_line("camera 相对 world         ", euler_zyx_intrinsic_deg(R_c2w));

    std::cout << "cam_origin_world_m: " << T_w_c.translation().transpose()
              << " | v=" << hero_board.bullet_speed
              << " | board_raw_deg yaw=" << static_cast<double>(snap.high_gimbal_yaw) * kRad2Deg
              << " pitch=" << static_cast<double>(snap.pitch) * kRad2Deg << "\n";
    std::cout << "---\n";

    if (!no_viz) {
      const Eigen::Vector3d gdeg = euler_zyx_intrinsic_deg(R_g2w);
      cv::Mat yaw_img = make_yaw_xy_panel(R_g2w, gdeg);
      cv::Mat pitch_img = make_pitch_xz_panel(R_g2w, gdeg);
      cv::imshow("hero_tf yaw (world XY)", yaw_img);
      cv::imshow("hero_tf pitch (world XZ)", pitch_img);
      cv::waitKey(1);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!no_viz) {
    cv::destroyAllWindows();
  }
  rclcpp::shutdown();
  return 0;
}
