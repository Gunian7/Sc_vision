/**
 * @brief 流程对齐 tests/camera_detect_test：仅图像来源为 ROS（ImageFromRos）。
 *
 * 每帧在得到装甲列表后对每块装甲做 PnP（Solver::solve），并在独立窗口 pnp_result 显示重投影。
 * 云台姿态固定为单位四元数（与无下位机标定自检一致）。
 *
 * 用法示例：
 *   ./build/hero_detectwithpnp_test configs/hero.yaml
 *   ./build/hero_detectwithpnp_test configs/hero.yaml --tradition=true --ros_topic=/image_for_auto_aim
 * Phoenix 传统检测仅由 yaml 根键 use_phoenix_traditional 控制，无 CLI 覆盖。
 * 传统路径（Phoenix 或 --tradition）下 yaml 根键 debug_img:true 可显示二值化调试图（imshow）。
 * Plotter：与 tests/camera_track_test 相同，从 yaml 读取 plotter_host / plotter_port（缺省 10.2.20.200:9870），
 *   每帧 UDP 发送 armor_num 与首块装甲位姿；--no_plotter=true 可关闭。
 *
 * 检测路径优先级：use_phoenix_traditional（yaml）> --tradition > YOLO。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <list>
#include <memory>

#include <Eigen/Geometry>
#include <fmt/core.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "io/image_from_ros/image_from_ros.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/phoenix_tradition_detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/hero_armor_detect.hpp"
#include "tools/hero_yolo_roi_log.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

static const std::string kCliKeys =
    "{help h usage ? | | 输出帮助}"
    "{@config-path | configs/hero.yaml | yaml 配置文件路径}"
    "{tradition t | false | 是否使用传统方法识别（Sc_vision 旧 Detector；Phoenix 开启时无效）}"
    "{ros_topic | /image_for_auto_aim | ROS 图像话题（与 camera_detect_test 唯一额外参数）}"
    "{queue q | 3 | 订阅队列深度}"
    "{no_plotter | false | 关闭 UDP Plotter（与 camera_track_test 一致的可由 yaml 配置 host/port）}";

int main(int argc, char** argv)
{
  cv::CommandLineParser cli(argc, argv, kCliKeys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>(0);
  const auto use_tradition = cli.get<bool>("tradition");
  const std::string image_topic = cli.get<std::string>("ros_topic");
  const int queue_cap = std::max(1, cli.get<int>("queue"));
  const bool no_plotter = cli.get<bool>("no_plotter");

  std::string plotter_host = "10.2.20.200";
  int plotter_port = 9870;
  const auto armor_yaml = tools::hero_load_armor_detect_yaml_flags(config_path);
  const bool use_phoenix_traditional = armor_yaml.use_phoenix_traditional;
  const bool debug_img = armor_yaml.debug_img;
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

  const bool tradition_debug =
    tools::hero_armor_tradition_visual_debug(debug_img, use_phoenix_traditional, use_tradition);

  rclcpp::init(argc, argv);

  tools::Exiter exiter;
  std::unique_ptr<tools::Plotter> plotter;
  if (!no_plotter) {
    tools::logger()->info("Plotter UDP -> {}:{}", plotter_host, plotter_port);
    plotter = std::make_unique<tools::Plotter>(
        plotter_host, static_cast<uint16_t>(plotter_port));
  }
  io::ImageFromRos camera(image_topic, static_cast<std::size_t>(queue_cap), false);

  auto_aim::Detector detector(config_path, tradition_debug);
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::PhoenixTraditionDetector phoenix_detector(
      config_path,
      tools::hero_phoenix_detector_impl_debug(tradition_debug, use_phoenix_traditional));
  auto_aim::Solver solver(config_path);

  const Eigen::Quaterniond identity_q = Eigen::Quaterniond::Identity();

  std::chrono::steady_clock::time_point timestamp;
  int total_frames = 0;
  int detect_frames = 0;

  tools::logger()->info(
      "hero_detectwithpnp: use_phoenix_traditional={} tradition_cli={} debug_img={}",
      use_phoenix_traditional,
      use_tradition,
      tradition_debug);

  while (!exiter.exit() && rclcpp::ok()) {
    cv::Mat img;
    std::list<auto_aim::Armor> armors;

    camera.read(img, timestamp);
    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    armors = tools::hero_detect_armors_for_frame(
        img, -1, armor_yaml, use_phoenix_traditional, use_tradition, phoenix_detector, detector, yolo);

    if (tools::hero_armor_detect_using_yolo_path(use_phoenix_traditional, use_tradition)) {
      tools::hero_log_yolo_roi(yolo);
    }

    solver.set_R_gimbal2world(identity_q);

    cv::Mat pnp_viz = img.clone();
    if (tools::hero_armor_detect_using_yolo_path(use_phoenix_traditional, use_tradition)) {
      tools::hero_draw_yolo_roi_overlay(pnp_viz, yolo);
    }
    if (!armors.empty()) {
      for (auto& a : armors) {
        solver.solve(a);
        const std::vector<cv::Point2f> pts = solver.reproject_armor(
            a.xyz_in_world, a.ypr_in_world[0], a.type, a.name);
        if (pts.size() == 4) {
          tools::draw_points(pnp_viz, pts, {0, 255, 255});
          cv::Point2f c{0.F, 0.F};
          for (const auto& p : pts) {
            c.x += p.x;
            c.y += p.y;
          }
          c.x /= 4.F;
          c.y /= 4.F;
          tools::draw_text(
              pnp_viz,
              fmt::format(
                  "{} {:.2f}m", auto_aim::ARMOR_NAMES[static_cast<int>(a.name)].c_str(),
                  a.xyz_in_gimbal.norm()),
              {std::max(4, static_cast<int>(c.x) - 30), std::max(16, static_cast<int>(c.y) - 10)},
              {255, 255, 255},
              0.55,
              2);
        }
      }
    } else {
      tools::draw_text(pnp_viz, "no armor", {10, 30}, {0, 0, 255}, 0.7, 2);
    }

    if (plotter) {
      nlohmann::json data;
      data["armor_num"] = armors.size();
      if (!armors.empty()) {
        const auto& armor = armors.front();
        data["armor_x"] = armor.xyz_in_world[0];
        data["armor_y"] = armor.xyz_in_world[1];
        data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
        if (std::isnan(armor.yaw_raw)) {
          data["armor_yaw_raw"] = nullptr;
        } else {
          data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
        }
      }
      plotter->plot(data);
    }

    cv::imshow("pnp_result", pnp_viz);

    total_frames++;
    if (!armors.empty()) detect_frames++;

    const double detect_rate = total_frames > 0 ? 100.0 * detect_frames / total_frames : 0.0;
    const auto now = std::chrono::steady_clock::now();
    const auto dt = tools::delta_time(now, last);
    tools::logger()->info(
        "{:.2f} fps  detect_rate={:.1f}%  ({}/{} frames)", 1 / dt, detect_rate, detect_frames,
        total_frames);

    // waitKey(33) 会每帧最多睡 33ms，显示与日志 fps 被人为压低、观感卡顿；1ms 仅用于刷新 imshow 事件循环
    const int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q') break;
  }

  const double final_rate = total_frames > 0 ? 100.0 * detect_frames / total_frames : 0.0;
  tools::logger()->info(
      "=== 识别结束: 总识别率 {:.2f}%  ({}/{} 帧) ===", final_rate, detect_frames, total_frames);

  rclcpp::shutdown();
  return 0;
}
