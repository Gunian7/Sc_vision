/**
 * Hero：主流程 + 扩展调试输出（多装甲、耗时分解、模式与弹速、可选 planner 细节）。
 */
#include <algorithm>
#include <chrono>
#include <list>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

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
#include "io/command.hpp"
#include "tools/exiter.hpp"
#include "tools/hero_armor_detect.hpp"
#include "tools/hero_auto_aim_viz.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

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

  rclcpp::init(argc, argv);

  tools::logger()->info(
      "[hero_full_test] 调试增强版 Hero 主程序 | use_phoenix_traditional={} tradition_cli={}",
      use_phoenix_traditional,
      tradition_cli);

  tools::Exiter exiter;
  std::unique_ptr<tools::Plotter> plotter;
  tools::HeroVizConfig vizcfg;
  vizcfg.window = !no_viz;
  vizcfg.plotter = !no_viz;
  if (vizcfg.plotter) {
    plotter = std::make_unique<tools::Plotter>();
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

  while (!exiter.exit() && rclcpp::ok()) {
    camera.read(img, t);
    q = hero_board.imu_at(t - std::chrono::milliseconds(1));

    tools::logger()->debug(
        "[hero_full_test] #{} bullet_v={:.2f} img={}x{}",
        frame_count,
        hero_board.bullet_speed,
        img.cols,
        img.rows);

    hero_solver.set_board_orientation(q);
    hero_solver.apply_to_solver(solver);

    const auto detect_start = std::chrono::steady_clock::now();
    auto armors = tools::hero_detect_armors_for_frame(
        img,
        frame_count,
        armor_yaml,
        use_phoenix_traditional,
        tradition_cli,
        phoenix_detector,
        legacy_detector,
        yolo);
    if (tools::hero_armor_detect_using_yolo_path(use_phoenix_traditional, tradition_cli)) {
      tools::hero_log_yolo_roi(yolo);
    }
    prioritize_outpost(armors);

    int ai = 0;
    for (const auto& a : armors) {
      tools::logger()->info(
          "[hero_full_test] det[{}] {} type={} pri={} conf={:.2f} box=({},{} {}x{}) center=({:.1f},{:.1f})",
          ai,
          auto_aim::ARMOR_NAMES[static_cast<int>(a.name)].c_str(),
          auto_aim::ARMOR_TYPES[static_cast<int>(a.type)].c_str(),
          static_cast<int>(a.priority),
          a.confidence,
          a.box.x,
          a.box.y,
          a.box.width,
          a.box.height,
          a.center.x,
          a.center.y);
      ++ai;
    }

    const auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, t);

    const auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, t, hero_board.bullet_speed);
    io::Command cmd_viz = targets.empty() ? io::neutral_command() : command;

    if (!targets.empty()) {
      const auto plan = planner.plan(targets.front(), hero_board.bullet_speed);
      tools::logger()->info(
          "[hero_full_test] planner control={} yaw={:.4f}rad pitch={:.4f}rad yaw_vel={:.4f} pitch_vel={:.4f}",
          plan.control,
          plan.yaw,
          plan.pitch,
          plan.yaw_vel,
          plan.pitch_vel);
      if (plan.control) {
        command.yaw = plan.yaw;
        command.pitch = plan.pitch;
        command.yaw_vel = plan.yaw_vel;
        command.pitch_vel = plan.pitch_vel;
      }
      cmd_viz = command;
    }

    const auto finish = std::chrono::steady_clock::now();

    if (!targets.empty()) {
      int ti = 0;
      for (const auto& tg : targets) {
        const Eigen::VectorXd x = tg.ekf_x();
        tools::logger()->info(
            "[hero_full_test] track[{}] {} motion={} jumped={} xyza_blocks={} | "
            "ekf x={:.3f} vx={:.3f} y={:.3f} vy={:.3f} z={:.3f} vz={:.3f} w={:.3f}",
            ti,
            auto_aim::ARMOR_NAMES[static_cast<int>(tg.name)].c_str(),
            static_cast<int>(tg.motion_state()),
            tg.jumped,
            tg.armor_xyza_list().size(),
            x[0],
            x[1],
            x[2],
            x[3],
            x[4],
            x[5],
            x[7]);
        ++ti;
      }

      const auto& tg = targets.front();
      const Eigen::VectorXd x = tg.ekf_x();
      tools::logger()->info(
          "[hero_full_test] #{} primary={} control={} shoot={} "
          "cmd_yaw={:.3f}deg cmd_pitch={:.3f}deg yaw_vel={:.3f} pitch_vel={:.3f} | "
          "detect {:.1f}ms tracker {:.1f}ms aimer+plan {:.1f}ms | "
          "ekf x={:.3f} vx={:.3f} y={:.3f} vy={:.3f} z={:.3f} vz={:.3f} w={:.3f}",
          frame_count,
          auto_aim::ARMOR_NAMES[static_cast<int>(tg.name)].c_str(),
          command.control,
          command.shoot,
          command.yaw * 57.3,
          command.pitch * 57.3,
          command.yaw_vel,
          command.pitch_vel,
          tools::delta_time(tracker_start, detect_start) * 1e3,
          tools::delta_time(aimer_start, tracker_start) * 1e3,
          tools::delta_time(finish, aimer_start) * 1e3,
          x[0],
          x[1],
          x[2],
          x[3],
          x[4],
          x[5],
          x[7]);
    } else {
      tools::logger()->info(
          "[hero_full_test] #{} no_track armors={} tracker.state={} | detect {:.1f}ms tracker {:.1f}ms",
          frame_count,
          armors.size(),
          tracker.state(),
          tools::delta_time(tracker_start, detect_start) * 1e3,
          tools::delta_time(aimer_start, tracker_start) * 1e3);
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
            vizcfg)) {
      break;
    }

    frame_count++;
  }

  rclcpp::shutdown();
  return 0;
}
