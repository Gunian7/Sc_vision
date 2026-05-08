/**
 * Hero：ROS 图 + YOLO + 每块装甲 PnP（内参来自 yaml），打印距离与重投影误差，用于核对 camera_matrix。
 * 用法与 hero 相同：-c yaml -t 图像话题
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <list>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "io/hero_ros_board/hero_ros_board.hpp"
#include "io/hero_ros_yaml_flags.hpp"
#include "io/image_from_ros/image_from_ros.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/hero_solver.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

namespace {

void prioritize_outpost(std::list<auto_aim::Armor>& armors)
{
  for (auto& a : armors) {
    if (a.name == auto_aim::ArmorName::outpost) {
      a.priority = auto_aim::ArmorPriority::first;
    } else {
      const int p = static_cast<int>(a.priority);
      const int second = static_cast<int>(auto_aim::ArmorPriority::second);
      if (p < second) {
        a.priority = auto_aim::ArmorPriority::second;
      }
    }
  }
}

double mean_reprojection_px(const auto_aim::Solver& solver, const auto_aim::Armor& armor)
{
  const std::vector<cv::Point2f> proj = solver.reproject_armor(
      armor.xyz_in_world, armor.ypr_in_world[0], armor.type, armor.name);
  if (proj.size() != 4 || armor.points.size() != 4) {
    return -1.;
  }
  double s = 0.;
  for (int i = 0; i < 4; ++i) {
    s += cv::norm(proj[static_cast<std::size_t>(i)] - armor.points[static_cast<std::size_t>(i)]);
  }
  return s / 4.0;
}

} // namespace

int main(int argc, char** argv)
{
  const std::string keys =
      "{help h usage ? | | 输出帮助}"
      "{config c | configs/hero.yaml | YAML}"
      "{topic t | /image_for_auto_aim | 图像话题}"
      "{queue q | 3 | 队列深度}";

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const std::string config_path = cli.get<std::string>("config");
  const std::string image_topic = cli.get<std::string>("topic");
  const int queue_cap = std::max(1, cli.get<int>("queue"));

  rclcpp::init(argc, argv);
  tools::Exiter exiter;
  io::HeroRosBoard hero_board(config_path);
  io::ImageFromRos camera(
      image_topic,
      static_cast<std::size_t>(queue_cap),
      io::yaml_hero_ros_suppress_rx_stall_log(config_path));

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::HeroSolver hero_solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  int frame_count = 0;

  tools::logger()->info(
      "[hero_detectwithpnp_test] PnP/距离/重投影自检；world 距离依赖当前板端姿态");

  while (!exiter.exit() && rclcpp::ok()) {
    camera.read(img, t);
    q = hero_board.imu_at(t - std::chrono::milliseconds(1));

    hero_solver.set_board_orientation(q);
    const io::HeroBoardParsed snap = hero_board.snapshot();
    if (snap.has_sample) {
      hero_solver.set_joint_pitch_rad(static_cast<double>(snap.vtx_pitch));
    }
    hero_solver.apply_to_solver(solver);

    auto armors = detector.detect(img, frame_count);
    prioritize_outpost(armors);

    if (armors.empty()) {
      tools::logger()->info("[hero_detectwithpnp_test] #{} no armor", frame_count);
      frame_count++;
      continue;
    }

    for (auto& a : armors) {
      solver.solve(a);
      const double d_cam = a.xyz_in_gimbal.norm();
      const double d_world = a.xyz_in_world.norm();
      const double d_ypd = a.ypd_in_world[2];
      const double reproj = mean_reprojection_px(solver, a);
      tools::logger()->info(
          "[hero_detectwithpnp_test] #{} {} type={} conf={:.2f} | "
          "d_gimbal_m={:.3f} d_world_m={:.3f} ypd_dist_m={:.3f} | "
          "ypr_world_deg=({:.2f},{:.2f},{:.2f}) reproj_px={:.3f}",
          frame_count,
          auto_aim::ARMOR_NAMES[static_cast<int>(a.name)].c_str(),
          auto_aim::ARMOR_TYPES[static_cast<int>(a.type)].c_str(),
          a.confidence,
          d_cam,
          d_world,
          d_ypd,
          a.ypr_in_world[0] * 57.3,
          a.ypr_in_world[1] * 57.3,
          a.ypr_in_world[2] * 57.3,
          reproj);
    }

    frame_count++;
  }

  rclcpp::shutdown();
  return 0;
}
