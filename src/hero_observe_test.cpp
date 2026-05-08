/**
 * Hero：跑完整观测链路（检测→跟踪→EKF/IMM 状态），逐帧打印装甲观测与目标内部状态。
 */
#include <algorithm>
#include <chrono>
#include <list>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "io/hero_ros_board/hero_ros_board.hpp"
#include "io/hero_ros_yaml_flags.hpp"
#include "io/image_from_ros/image_from_ros.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/hero_solver.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

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
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  int frame_count = 0;

  tools::logger()->info("[hero_observe_test] 全观测输出（tracker 状态 / EKF / 装甲列表）");

  while (!exiter.exit() && rclcpp::ok()) {
    camera.read(img, t);
    q = hero_board.imu_at(t - std::chrono::milliseconds(1));

    hero_solver.set_board_orientation(q);
    const io::HeroBoardParsed snap = hero_board.snapshot();
    if (snap.has_sample) {
      hero_solver.set_joint_pitch_rad(static_cast<double>(snap.vtx_pitch));
    }
    hero_solver.apply_to_solver(solver);

    const auto t0 = std::chrono::steady_clock::now();
    auto armors = detector.detect(img, frame_count);
    prioritize_outpost(armors);
    const auto t1 = std::chrono::steady_clock::now();

    for (const auto& a : armors) {
      auto_aim::Armor acopy = a;
      solver.solve(acopy);
      tools::logger()->info(
          "[observe] armor id={} {} conf={:.2f} | world_m=({:.3f},{:.3f},{:.3f}) "
          "yaw_deg={:.2f} pitch_deg={:.2f} center_px=({:.1f},{:.1f})",
          a.class_id,
          auto_aim::ARMOR_NAMES[static_cast<int>(a.name)].c_str(),
          a.confidence,
          acopy.xyz_in_world[0],
          acopy.xyz_in_world[1],
          acopy.xyz_in_world[2],
          acopy.ypr_in_world[0] * 57.3,
          acopy.ypr_in_world[1] * 57.3,
          a.center.x,
          a.center.y);
    }

    auto targets = tracker.track(armors, t);
    const auto t2 = std::chrono::steady_clock::now();

    tools::logger()->info(
        "[observe] #{} tracker.state={} armors={} targets={} | yolo {:.1f}ms track {:.1f}ms",
        frame_count,
        tracker.state(),
        armors.size(),
        targets.size(),
        tools::delta_time(t1, t0) * 1e3,
        tools::delta_time(t2, t1) * 1e3);

    int ti = 0;
    for (const auto& tg : targets) {
      const Eigen::VectorXd x = tg.ekf_x();
      const auto xyza = tg.armor_xyza_list();
      tools::logger()->info(
          "[observe] target[{}] name={} jumped={} motion={} imm_w={:.4f} imm_a={:.4f} | "
          "ekf x={:.3f} vx={:.3f} y={:.3f} vy={:.3f} z={:.3f} vz={:.3f} w={:.3f} | xyza_n={}",
          ti,
          auto_aim::ARMOR_NAMES[static_cast<int>(tg.name)].c_str(),
          tg.jumped,
          static_cast<int>(tg.motion_state()),
          tg.imm_w(),
          tg.imm_alpha(),
          x[0],
          x[1],
          x[2],
          x[3],
          x[4],
          x[5],
          x[7],
          xyza.size());
      int ai = 0;
      for (const auto& row : xyza) {
        tools::logger()->info(
            "    xyza[{}]=({:.3f},{:.3f},{:.3f},{:.3f})",
            ai,
            row[0],
            row[1],
            row[2],
            row[3]);
        ++ai;
      }
      ++ti;
    }

    if (!targets.empty()) {
      auto command = aimer.aim(targets, t, hero_board.bullet_speed);
      const auto plan = planner.plan(targets.front(), hero_board.bullet_speed);
      tools::logger()->info(
          "[observe] cmd yaw={:.3f}deg pitch={:.3f}deg ctrl={} shoot={} | plan.ctrl={}",
          command.yaw * 57.3,
          command.pitch * 57.3,
          command.control,
          command.shoot,
          plan.control);
    }

    frame_count++;
  }

  rclcpp::shutdown();
  return 0;
}
