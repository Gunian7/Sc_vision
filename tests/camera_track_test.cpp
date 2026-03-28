/**
 * @brief 相机 + 追踪器 + Aimer 在线测试
 *
 * 不需要 CBoard / 串口，IMU 姿态固定为单位四元数（等效云台不动），
 * bullet_speed 使用 yaml 中配置的固定值。
 * 用于在无下位机环境下验证识别、追踪、瞄准逻辑。
 *
 * 用法：
 *   ./build/camera_track_test configs/sentry_blue.yaml
 */

#include <fmt/core.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml    | 位置参数，yaml配置文件路径}"
  "{speed s        | 15.0                   | 固定弹速 m/s}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path  = cli.get<std::string>(0);
  double bullet_speed = cli.get<double>("speed");
  std::string plotter_host = "10.2.20.200";
  int plotter_port = 9870;

  double jump_pitch_up_duration   = 0.0;
  double jump_pitch_down_duration = 0.0;
  double decision_speed           = 0.0;
  try {
    auto yaml = YAML::LoadFile(config_path);
    if (yaml["bullet_speed"].IsDefined())
      bullet_speed = yaml["bullet_speed"].as<double>();
    if (yaml["jump_pitch_up_duration"].IsDefined())
      jump_pitch_up_duration = yaml["jump_pitch_up_duration"].as<double>();
    if (yaml["jump_pitch_down_duration"].IsDefined())
      jump_pitch_down_duration = yaml["jump_pitch_down_duration"].as<double>();
    if (yaml["decision_speed"].IsDefined())
      decision_speed = yaml["decision_speed"].as<double>();
    if (yaml["plotter_host"].IsDefined())
      plotter_host = yaml["plotter_host"].as<std::string>();
    if (yaml["plotter_port"].IsDefined())
      plotter_port = yaml["plotter_port"].as<int>();
  } catch (...) {}

  tools::logger()->info("bullet_speed={:.1f} m/s  decision_speed={:.1f} rad/s (无下位机模式)",
                        bullet_speed, decision_speed);
  tools::logger()->info("Plotter UDP -> {}:{}", plotter_host, plotter_port);

  tools::Exiter exiter;
  tools::Plotter plotter(plotter_host, static_cast<uint16_t>(plotter_port));

  io::Camera camera(config_path);
  auto_aim::YOLO    yolo(config_path, true);
  auto_aim::Solver  solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer   aimer(config_path);

  // 固定单位四元数：等效云台水平静止，世界系 = 云台系
  const Eigen::Quaterniond identity_q = Eigen::Quaterniond::Identity();
  solver.set_R_gimbal2world(identity_q);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    if (img.empty()) break;

    solver.set_R_gimbal2world(identity_q);

    auto armors  = yolo.detect(img);
    cv::Rect roi_rect;
    bool roi_active = false;
    if (yolo.get_debug_roi(roi_rect, roi_active)) {
      tools::logger()->info("[ROI] active={}, x={}, y={}, w={}, h={}",
        roi_active, roi_rect.x, roi_rect.y, roi_rect.width, roi_rect.height);
    }
    auto targets = tracker.track(armors, t);
    auto command = aimer.aim(targets, t, bullet_speed);

    // --------- 终端日志 ---------
    if (!targets.empty()) {
      const auto & tgt = targets.front();
      auto ekf = tgt.ekf_x();
      tools::logger()->info(
        "[Track] state={} w={:.2f} rad/s  vx={:.2f} vy={:.2f} vz={:.2f}",
        tracker.state(),
        ekf[7], ekf[1], ekf[3], ekf[5]);
    } else {
      tools::logger()->info("[Track] state={} no target", tracker.state());
    }

    // --------- 可视化（与 standard_mpc_se 一致）---------
    tools::draw_text(
      img,
      fmt::format("cmd {},{:.2f},{:.2f},shoot:{}", command.control,
                  command.yaw * 57.3, command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    nlohmann::json data;
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor    = armors.front();
      data["armor_x"]       = armor.xyz_in_world[0];
      data["armor_y"]       = armor.xyz_in_world[1];
      data["armor_yaw"]     = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    }

    if (!targets.empty()) {
      auto target                                  = targets.front();
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();

      // 前哨站高度排序
      std::array<int, 3> outpost_order{0, 1, 2};
      if (target.name == auto_aim::ArmorName::outpost && armor_xyza_list.size() == 3) {
        std::sort(outpost_order.begin(), outpost_order.end(),
                  [&](int a, int b) { return armor_xyza_list[a][2] < armor_xyza_list[b][2]; });
      }

      // 跳变标签
      bool show_jump_up   = false;
      bool show_jump_down = false;
      if (target.has_jump_time()) {
        const double abs_w = std::abs(target.ekf_x()[7]);
        if (abs_w >= decision_speed) {
          auto age = std::chrono::duration<double>(t - target.last_jump_time()).count();
          auto dir = target.last_jump_dir();
          if (dir < 0 && jump_pitch_up_duration > 0.0 && age >= 0.0 && age <= jump_pitch_up_duration)
            show_jump_up = true;
          if (dir > 0 && jump_pitch_down_duration > 0.0 && age >= 0.0 && age <= jump_pitch_down_duration)
            show_jump_down = true;
        }
      }

      // 各装甲板重投影（绿色）+ id 标注
      for (std::size_t i = 0; i < armor_xyza_list.size(); ++i) {
        const auto & xyza   = armor_xyza_list[i];
        auto image_points   = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});

        cv::Point2f center{0.0F, 0.0F};
        for (const auto & pt : image_points) { center.x += pt.x; center.y += pt.y; }
        center.x /= static_cast<float>(image_points.size());
        center.y /= static_cast<float>(image_points.size());
        tools::draw_text(img, fmt::format("id:{}", i), center, {255, 255, 0});

        if (target.name == auto_aim::ArmorName::outpost && armor_xyza_list.size() == 3) {
          std::string tag = "middle";
          if (static_cast<int>(i) == outpost_order[0]) tag = "low";
          if (static_cast<int>(i) == outpost_order[2]) tag = "high";
          tools::draw_text(img, tag, {center.x, center.y + 18.0F}, {0, 255, 255});
        }
      }

      // 车心位置（黄色圆点）
      Eigen::VectorXd x = target.ekf_x();
      std::vector<cv::Point3f> center_pt = {{
        static_cast<float>(x[0]),
        static_cast<float>(x[2]),
        static_cast<float>(x[4])}};
      auto center_img_pts = solver.world2pixel(center_pt);
      if (!center_img_pts.empty()) {
        cv::circle(img, center_img_pts[0], 5, {255, 255, 0}, -1);
      }

      // 瞄准点重投影（红色）+ 跳变标签
      auto aim_point           = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto aim_image_points    =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) {
        tools::draw_points(img, aim_image_points, {0, 0, 255});

        cv::Point2f aim_center{0.0F, 0.0F};
        for (const auto & pt : aim_image_points) { aim_center.x += pt.x; aim_center.y += pt.y; }
        aim_center.x /= static_cast<float>(aim_image_points.size());
        aim_center.y /= static_cast<float>(aim_image_points.size());
        if (show_jump_up)
          tools::draw_text(img, "Up",   {aim_center.x, aim_center.y - 14.0F}, {0, 165, 255});
        if (show_jump_down)
          tools::draw_text(img, "Down", {aim_center.x, aim_center.y - 14.0F}, {0, 0, 255});
      }

      // PlotJuggler 数据
      data["x"]       = x[0]; data["vx"] = x[1];
      data["y"]       = x[2]; data["vy"] = x[3];
      data["z"]       = x[4]; data["vz"] = x[5];
      data["a"]       = x[6] * 57.3;
      data["w"]       = x[7];
      data["r"]       = x[8];
      data["l"]       = x[9];
      data["h"]       = x[10];
      data["last_id"] = target.last_id;

      auto ekf_debug = target.ekf();
      data["residual_yaw"]          = ekf_debug.data.at("residual_yaw");
      data["residual_pitch"]        = ekf_debug.data.at("residual_pitch");
      data["residual_distance"]     = ekf_debug.data.at("residual_distance");
      data["residual_angle"]        = ekf_debug.data.at("residual_angle");
      data["nis"]                   = ekf_debug.data.at("nis");
      data["nees"]                  = ekf_debug.data.at("nees");
      data["nis_fail"]              = ekf_debug.data.at("nis_fail");
      data["nees_fail"]             = ekf_debug.data.at("nees_fail");
      data["recent_nis_failures"]   = ekf_debug.data.at("recent_nis_failures");
    }
    plotter.plot(data);

    cv::resize(img, img, {}, 0.9, 0.9);
    cv::imshow("reprojection", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}
