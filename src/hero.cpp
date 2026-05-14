/**
 * Hero：ROS 取图（ImageFromRos）+ 与 standard 类似的自瞄流水线。
 * 下位机自瞄反馈经 HeroRosBoard 订阅 communicate Autoaim（默认 /communicate/autoaim），替代串口 CBoard。
 * 前哨站目标优先于普通装甲（在送入 Tracker 前调整 ArmorPriority）。
 * 不做能量机关；主流程不依据下位机 mode 门控，始终跑自瞄链路（视仅在 auto_aim 场景使用）。
 * 前哨战逻辑由装甲分类 ArmorName::outpost 触发。
 * 控制指令：默认仅打日志；hero_command_publish_enable:true 时发布 SerialInfo（默认 topic 见 hero.yaml）。若运行仓库 hero_common 的 hero_link_node，话题应对齐其 vision_serial_topic，由其 mux 到 communicate /shoot_info。
 */
#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <list>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "io/hero_config_path.hpp"
#include "io/hero_ros_board/hero_ros_board.hpp"
#include "io/hero_ros_board/hero_ros_command_pub.hpp"
#include "io/hero_ros_yaml_flags.hpp"
#include "io/image_from_ros/image_from_ros.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/hero_solver.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/phoenix_tradition_detector.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/hero_armor_detect.hpp"
#include "tools/hero_auto_aim_viz.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace
{

void prioritize_outpost(std::list<auto_aim::Armor> & armors)
{
  for (auto & a : armors) {
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

}  // namespace

int main(int argc, char ** argv)
{
  const std::string keys =
      "{help h usage ? | | 输出帮助}"
      "{config c | configs/hero.yaml | Hero 专用 YAML（可与 standard 对齐后按需改）}"
      "{topic t | /image_for_auto_aim | sensor_msgs/Image 话题}"
      "{queue q | 3 | 图像队列深度（满则丢最旧）}"
      "{tradition t | false | Sc_vision 传统 Detector；yaml use_phoenix_traditional 为 true 时不走此分支}"
      "{no_viz | false | 关闭 OpenCV 重投影窗口与 UDP plotter（无头/远程）}";

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

  tools::Exiter exiter;
  std::unique_ptr<tools::Plotter> plotter;
  tools::HeroVizConfig vizcfg;
  vizcfg.window = !no_viz;
  vizcfg.plotter = !no_viz;
  if (vizcfg.plotter) {
    plotter = std::make_unique<tools::Plotter>();
  }
  io::HeroRosBoard hero_board(config_path);
  io::HeroRosCommandPublisher hero_cmd_pub(config_path);
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

  tools::logger()->info(
      "[hero] armor_detect: use_phoenix_traditional={} tradition_cli={} debug_img={}",
      use_phoenix_traditional,
      tradition_cli,
      armor_yaml.debug_img);

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
    if (tools::hero_armor_detect_using_yolo_path(use_phoenix_traditional, tradition_cli)) {
      tools::hero_log_yolo_roi(yolo);
    }
    prioritize_outpost(armors);

    const auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, t);

    const auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, t, hero_board.bullet_speed);

    if (!targets.empty()) {
      const auto plan = planner.plan(targets.front(), hero_board.bullet_speed);
      if (plan.control) {
        command.yaw = plan.yaw;
        command.pitch = plan.pitch;
        command.yaw_vel = plan.yaw_vel;
        command.pitch_vel = plan.pitch_vel;
      }
    }

    const auto finish = std::chrono::steady_clock::now();

    const io::Command cmd_out = targets.empty() ? io::neutral_command() : command;
    if (hero_cmd_pub.enabled()) {
      hero_cmd_pub.publish(cmd_out);
    }

    if (!targets.empty()) {
      const auto & tg = targets.front();
      const Eigen::VectorXd x = tg.ekf_x();
      tools::logger()->info(
          "[hero] #{} target={} control={} shoot={} "
          "cmd_yaw={:.3f}deg cmd_pitch={:.3f}deg yaw_vel={:.3f} pitch_vel={:.3f} | "
          "detect {:.1f}ms tracker {:.1f}ms aimer {:.1f}ms | "
          "ekf x={:.3f} vx={:.3f} y={:.3f} vy={:.3f} z={:.3f} vz={:.3f} w={:.3f}",
          frame_count,
          auto_aim::ARMOR_NAMES[static_cast<int>(tg.name)],
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
          "[hero] #{} no_track armors={} | detect {:.1f}ms tracker {:.1f}ms",
          frame_count,
          armors.size(),
          tools::delta_time(tracker_start, detect_start) * 1e3,
          tools::delta_time(aimer_start, tracker_start) * 1e3);
    }

    if (tools::hero_viz_auto_aim_frame(
            img,
            solver,
            aimer,
            armors,
            targets,
            cmd_out,
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
