/**
 * Hero：无实机下位机场景。
 * - 推荐：communicate 节点 param serial_enable:false，由其对 /communicate/autoaim 发假数据；
 * - 或：configs/hero_withoutboard.yaml 中 hero_board_simulate_no_mcu:true（本地恒等姿态）。
 * 图像与识别链路与 hero 相同。
 */
#include <algorithm>
#include <chrono>
#include <list>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

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
      "{config c | configs/hero_withoutboard.yaml | YAML（默认无下位机本地模拟）}"
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
      "[hero_withoutboard_test] 无下位机：communicate serial_enable:false 或 yaml "
      "hero_board_simulate_no_mcu:true | config={} | use_phoenix_traditional={} tradition_cli={}",
      config_path,
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
    if (tools::hero_armor_detect_using_yolo_path(armor_yaml, use_phoenix_traditional, tradition_cli)) {
      tools::hero_log_yolo_roi(yolo);
    }
    prioritize_outpost(armors);

    const auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, t);

    const auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, t, hero_board.bullet_speed);
    io::Command cmd_viz = targets.empty() ? io::neutral_command() : command;

    if (!targets.empty()) {
      const auto plan = planner.plan(targets.front(), hero_board.bullet_speed);
      if (plan.control) {
        command.yaw = plan.yaw;
        command.pitch = plan.pitch;
        command.yaw_vel = plan.yaw_vel;
        command.pitch_vel = plan.pitch_vel;
      }
      cmd_viz = command;
    }

    if (!targets.empty()) {
      const auto& tg = targets.front();
      const Eigen::VectorXd x = tg.ekf_x();
      tools::logger()->info(
          "[hero_withoutboard_test] #{} target={} armors={} cmd_yaw={:.2f}deg cmd_pitch={:.2f}deg | "
          "detect {:.1f}ms tracker {:.1f}ms | ekf z={:.3f}",
          frame_count,
          auto_aim::ARMOR_NAMES[static_cast<int>(tg.name)].c_str(),
          armors.size(),
          command.yaw * 57.3,
          command.pitch * 57.3,
          tools::delta_time(tracker_start, detect_start) * 1e3,
          tools::delta_time(aimer_start, tracker_start) * 1e3,
          x[4]);
    } else {
      tools::logger()->info(
          "[hero_withoutboard_test] #{} no_track armors={} | detect {:.1f}ms",
          frame_count,
          armors.size(),
          tools::delta_time(tracker_start, detect_start) * 1e3);
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
