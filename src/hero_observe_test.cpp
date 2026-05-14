/**
 * Hero：跑完整观测链路（检测→跟踪→EKF/IMM 状态），逐帧打印装甲观测与目标内部状态。
 * Plotter：与 tests/camera_track_test 一致，从 yaml 读取 plotter_host / plotter_port（缺省 10.2.20.200:9870）；
 *   --no_viz 时同时关闭窗口与 UDP。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "io/command.hpp"
#include "io/hero_config_path.hpp"
#include "io/hero_ros_board/hero_ros_board.hpp"
#include "io/hero_ros_yaml_flags.hpp"
#include "io/image_from_ros/image_from_ros.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/hero_solver.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/phoenix_tradition_detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/hero_armor_detect.hpp"
#include "tools/hero_auto_aim_viz.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace {

const char* detect_path_name(bool use_phoenix_traditional, bool tradition_cli)
{
  if (use_phoenix_traditional) {
    return "phoenix_traditional";
  }
  if (tradition_cli) {
    return "legacy_traditional";
  }
  return "yolo";
}

bool armor_points_sane(const auto_aim::Armor& a, double& area_px2)
{
  if (a.points.size() != 4) {
    area_px2 = 0.0;
    return false;
  }
  for (const auto& p : a.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      area_px2 = 0.0;
      return false;
    }
  }
  area_px2 = std::abs(cv::contourArea(a.points));
  return area_px2 > 1.0;
}

bool finite_vec3(const Eigen::Vector3d& v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

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
      "{queue q | 3 | 队列深度}"
      "{tradition t | false | Sc_vision 传统 Detector；yaml use_phoenix_traditional 为 true 时不走此分支}"
      "{no_viz | false | 关闭 OpenCV 重投影与 UDP plotter}";

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const std::string config_path =
      io::resolve_config_path_next_to_build(cli.get<std::string>("config"), argv[0]);
  const std::string image_topic = cli.get<std::string>("topic");
  const int queue_cap = std::max(1, cli.get<int>("queue"));
  const bool tradition_cli = cli.get<bool>("tradition");
  const bool no_viz = cli.get<bool>("no_viz");

  const auto armor_yaml = tools::hero_load_armor_detect_yaml_flags(config_path);
  const bool use_phoenix_traditional = armor_yaml.use_phoenix_traditional;
  const bool tradition_visual = tools::hero_armor_tradition_visual_debug(
      armor_yaml.debug_img, use_phoenix_traditional, tradition_cli);

  std::string plotter_host = "10.2.20.200";
  int plotter_port = 9870;
  try {
    auto yaml = YAML::LoadFile(config_path);
    if (yaml["plotter_host"].IsDefined()) {
      plotter_host = yaml["plotter_host"].as<std::string>();
    }
    if (yaml["plotter_port"].IsDefined()) {
      plotter_port = yaml["plotter_port"].as<int>();
    }
  } catch (...) {
  }

  rclcpp::init(argc, argv);
  tools::Exiter exiter;
  std::unique_ptr<tools::Plotter> plotter;
  tools::HeroVizConfig vizcfg;
  vizcfg.window = !no_viz;
  vizcfg.plotter = !no_viz;
  if (vizcfg.plotter) {
    tools::logger()->info("Plotter UDP -> {}:{}", plotter_host, plotter_port);
    plotter = std::make_unique<tools::Plotter>(
        plotter_host, static_cast<uint16_t>(plotter_port));
  }
  io::HeroRosBoard hero_board(config_path);
  io::ImageFromRos camera(
      image_topic,
      static_cast<std::size_t>(queue_cap),
      io::yaml_hero_ros_suppress_rx_stall_log(config_path));

  auto_aim::Detector legacy_detector(config_path, tradition_visual);
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::PhoenixTraditionDetector phoenix_detector(
      config_path,
      tools::hero_phoenix_detector_impl_debug(tradition_visual, use_phoenix_traditional));
  auto_aim::Solver solver(config_path);
  auto_aim::HeroSolver hero_solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  int frame_count = 0;
  int detect_empty_streak = 0;
  int target_empty_streak = 0;
  std::string last_tracker_state = tracker.state();

  tools::logger()->info(
      "[hero_observe_test] 全观测输出（tracker 状态 / EKF / 装甲列表）| "
      "use_phoenix_traditional={} tradition_cli={} detect_path={}",
      use_phoenix_traditional,
      tradition_cli,
      detect_path_name(use_phoenix_traditional, tradition_cli));

  while (!exiter.exit() && rclcpp::ok()) {
    camera.read(img, t);
    q = hero_board.imu_at(t - std::chrono::milliseconds(1));

    hero_solver.set_board_orientation(q);
    hero_solver.apply_to_solver(solver);

    const auto t0 = std::chrono::steady_clock::now();
    auto armors = tools::hero_detect_armors_for_frame(
        img,
        frame_count,
        armor_yaml,
        use_phoenix_traditional,
        tradition_cli,
        phoenix_detector,
        legacy_detector,
        yolo);
    if (tools::hero_armor_detect_using_yolo_path(armor_yaml, use_phoenix_traditional, tradition_cli)) {
      tools::hero_log_yolo_roi(yolo);
    }
    prioritize_outpost(armors);
    const auto t1 = std::chrono::steady_clock::now();

    if (frame_count % 60 == 0) {
      const auto snap = hero_board.snapshot();
      tools::logger()->info(
          "[observe][board] has_sample={} mode={} bullet_speed={:.2f} "
          "raw_yaw={:.3f} raw_pitch={:.3f}",
          snap.has_sample,
          io::MODES[hero_board.mode].c_str(),
          hero_board.bullet_speed,
          snap.high_gimbal_yaw,
          snap.pitch);
    }

    if (armors.empty() && frame_count % 30 == 0) {
      tools::logger()->warn(
          "[observe][detect] armors=0 path={} dt={:.2f}ms (连续空帧请重点排查阈值/模型置信度/输入图像)",
          detect_path_name(use_phoenix_traditional, tradition_cli),
          tools::delta_time(t1, t0) * 1e3);
    }

    // 与当前云台姿态一致的世界系量；tracker.track 内仍会对匹配装甲再次 solve（结果应一致）。
    int valid_pnp_count = 0;
    for (auto & a : armors) {
      double area_px2 = 0.0;
      if (!armor_points_sane(a, area_px2)) {
        tools::logger()->warn(
            "[observe][pnp] skip invalid armor points: id={} conf={:.2f} points_n={}",
            a.class_id,
            a.confidence,
            a.points.size());
        continue;
      }
      solver.solve(a);
      if (!finite_vec3(a.xyz_in_world) || !finite_vec3(a.ypr_in_world) || !finite_vec3(a.ypd_in_world)) {
        tools::logger()->warn(
            "[observe][pnp] non-finite result: id={} conf={:.2f} area_px2={:.1f}",
            a.class_id,
            a.confidence,
            area_px2);
        continue;
      }
      ++valid_pnp_count;
      const Eigen::Vector3d ypd = a.ypd_in_world;
      tools::logger()->info(
          "[observe] armor id={} {} conf={:.2f} | world_m=({:.3f},{:.3f},{:.3f}) "
          "ypr_deg(yaw,pitch)=({:.2f},{:.2f}) ypd_deg(y,p)=({:.2f},{:.2f}) dist_m={:.3f} "
          "center_px=({:.1f},{:.1f}) area_px2={:.1f}",
          a.class_id,
          auto_aim::ARMOR_NAMES[static_cast<int>(a.name)].c_str(),
          a.confidence,
          a.xyz_in_world[0],
          a.xyz_in_world[1],
          a.xyz_in_world[2],
          a.ypr_in_world[0] * 57.3,
          a.ypr_in_world[1] * 57.3,
          ypd[0] * 57.3,
          ypd[1] * 57.3,
          ypd[2],
          a.center.x,
          a.center.y,
          area_px2);
    }

    auto targets = tracker.track(armors, t);
    const auto t2 = std::chrono::steady_clock::now();
    const std::string curr_tracker_state = tracker.state();

    if (curr_tracker_state != last_tracker_state) {
      tools::logger()->warn(
          "[observe][track] state {} -> {} | armors={} valid_pnp={} targets={} detect_empty_streak={} target_empty_streak={}",
          last_tracker_state,
          curr_tracker_state,
          armors.size(),
          valid_pnp_count,
          targets.size(),
          detect_empty_streak,
          target_empty_streak);
      last_tracker_state = curr_tracker_state;
    }

    tools::logger()->info(
        "[observe] #{} tracker.state={} armors={} targets={} | detect {:.1f}ms track {:.1f}ms",
        frame_count,
        tracker.state(),
        armors.size(),
        targets.size(),
        tools::delta_time(t1, t0) * 1e3,
        tools::delta_time(t2, t1) * 1e3);

    detect_empty_streak = armors.empty() ? (detect_empty_streak + 1) : 0;
    target_empty_streak = targets.empty() ? (target_empty_streak + 1) : 0;

    if (!armors.empty() && targets.empty()) {
      tools::logger()->warn(
          "[observe][track] detection->track drop: armors={} valid_pnp={} state={} (可能是匹配门限/状态机约束导致未成 target)",
          armors.size(),
          valid_pnp_count,
          curr_tracker_state);
    }
    if (armors.empty() && !targets.empty() && detect_empty_streak % 15 == 0) {
      tools::logger()->info(
          "[observe][track] predict-only: detect_empty_streak={} targets={} state={}",
          detect_empty_streak,
          targets.size(),
          curr_tracker_state);
    }
    if (detect_empty_streak > 0 && detect_empty_streak % 30 == 0) {
      tools::logger()->warn(
          "[observe][diag] 连续空检测 {} 帧 | tracker_state={} targets={} target_empty_streak={}",
          detect_empty_streak,
          curr_tracker_state,
          targets.size(),
          target_empty_streak);
    }

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

    io::Command cmd_viz = io::neutral_command();
    if (!targets.empty()) {
      auto command = aimer.aim(targets, t, hero_board.bullet_speed);
      const auto plan = planner.plan(targets.front(), hero_board.bullet_speed);
      if (plan.control) {
        command.yaw = plan.yaw;
        command.pitch = plan.pitch;
        command.yaw_vel = plan.yaw_vel;
        command.pitch_vel = plan.pitch_vel;
      }
      cmd_viz = command;
      tools::logger()->info(
          "[observe] cmd yaw={:.3f}deg pitch={:.3f}deg ctrl={} shoot={} | plan.ctrl={}",
          command.yaw * 57.3,
          command.pitch * 57.3,
          command.control,
          command.shoot,
          plan.control);
    }

    if (vizcfg.window &&
        tools::hero_armor_detect_using_yolo_path(armor_yaml, use_phoenix_traditional, tradition_cli)) {
      tools::hero_draw_yolo_roi_overlay(img, yolo);
    }

    if (tools::hero_viz_auto_aim_frame(
            img,
            solver,
            aimer,
            armors,
            targets,
            cmd_viz,
            q,
            plotter.get(),
            vizcfg,
            curr_tracker_state)) {
      break;
    }

    frame_count++;
  }

  rclcpp::shutdown();
  return 0;
}
