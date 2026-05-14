#ifndef TOOLS__HERO_AUTO_AIM_VIZ_HPP
#define TOOLS__HERO_AUTO_AIM_VIZ_HPP

/**
 * Hero 系列 ROS 取图程序的可视化：与 standard / standard_mpc_se 一致的重投影 + plotter + imshow。
 */
#include <algorithm>
#include <cmath>
#include <list>
#include <string>

#include <Eigen/Geometry>
#include <fmt/core.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/hero_yolo_roi_log.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace tools
{

struct HeroVizConfig
{
  bool window{true};
  bool plotter{true};
  double resize_scale{0.9};
  /** hero_viz_pnp_armors_frame：绿色检测角点 armor.points */
  bool pnp_viz_green{true};
  /** hero_viz_pnp_armors_frame：黄色 PnP 重投影 */
  bool pnp_viz_yellow{true};
};

/**
 * @return true 表示用户按了 q，主循环应退出
 */
inline bool hero_viz_auto_aim_frame(
  cv::Mat & img,
  auto_aim::Solver & solver,
  const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Armor> & armors,
  const std::list<auto_aim::Target> & targets,
  const io::Command & command,
  const Eigen::Quaterniond & gimbal_q,
  Plotter * plotter,
  const HeroVizConfig & cfg,
  const std::string & tracker_state = "")
{
  const bool need_draw = cfg.window || (cfg.plotter && plotter != nullptr);
  if (!need_draw) {
    return false;
  }

  Eigen::Vector3d ypr = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0);
  const double yaw = ypr[0];

  if (cfg.window) {
    if (armors.empty()) {
      tools::draw_text(img, "no armor detected", {10, 30}, {0, 0, 255});
    }
    tools::draw_text(
      img,
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}",
        command.control,
        command.yaw * 57.3,
        command.pitch * 57.3,
        command.shoot),
      {10, 60},
      {154, 50, 205});
    tools::draw_text(
      img,
      fmt::format("gimbal yaw{:.2f}", yaw * 57.3),
      {10, 90},
      {255, 255, 255});
    if (!tracker_state.empty()) {
      tools::draw_text(
        img,
        fmt::format("tracker state: {}", tracker_state),
        {10, 118},
        {180, 255, 180},
        0.5,
        2);
    }

    // 检测框：与 Detector::show_result 一致写出装甲名 + 大小类型（EKF 绿框在上层绘制）
    std::string detect_summary = "detect:";
    int si = 0;
    for (const auto & armor : armors) {
      const char * nm = auto_aim::ARMOR_NAMES[static_cast<int>(armor.name)].c_str();
      const char * tp = auto_aim::ARMOR_TYPES[static_cast<int>(armor.type)].c_str();
      cv::Point2f anchor = armor.center;
      if (armor.points.size() == 4) {
        anchor = {0.F, 0.F};
        for (const auto & p : armor.points) {
          anchor.x += p.x;
          anchor.y += p.y;
        }
        anchor.x *= 0.25F;
        anchor.y *= 0.25F;
        tools::draw_points(img, armor.points, {0, 165, 255}, 2);
      } else if (armor.box.width > 0 && armor.box.height > 0) {
        anchor.x = armor.box.x + armor.box.width * 0.5F;
        anchor.y = static_cast<float>(armor.box.y) - 4.F;
      }
      const int tx = std::max(4, static_cast<int>(anchor.x) - 48);
      const int ty = std::max(16, static_cast<int>(anchor.y) - 10);
      tools::draw_text(
        img,
        fmt::format("{} {} {:.2f}", nm, tp, armor.confidence),
        {tx, ty},
        {200, 255, 200},
        0.45,
        2);
      if (si < 6) {
        if (si++) {
          detect_summary += ",";
        }
        detect_summary += fmt::format("{}/{}", nm, tp);
      }
    }
    if (!armors.empty()) {
      if (armors.size() > 6U) {
        detect_summary += fmt::format(",+{}", armors.size() - 6U);
      }
      tools::draw_text(img, detect_summary, {10, 138}, {180, 220, 255}, 0.45, 1);
    }
  }

  nlohmann::json data;
  data["armor_num"] = armors.size();
  data["armors"] = nlohmann::json::array();
  for (const auto & armor : armors) {
    nlohmann::json a;
    a["name"] = auto_aim::ARMOR_NAMES[static_cast<int>(armor.name)];
    a["type"] = auto_aim::ARMOR_TYPES[static_cast<int>(armor.type)];
    a["color"] = auto_aim::COLORS[static_cast<int>(armor.color)];
    a["class_id"] = armor.class_id;
    a["priority"] = static_cast<int>(armor.priority);
    a["confidence"] = armor.confidence;

    a["xyz_world"] = {armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2]};
    a["xyz_gimbal"] = {armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], armor.xyz_in_gimbal[2]};
    a["ypr_world_deg"] = {
      armor.ypr_in_world[0] * 57.3, armor.ypr_in_world[1] * 57.3, armor.ypr_in_world[2] * 57.3};
    a["ypd_world"] = {armor.ypd_in_world[0], armor.ypd_in_world[1], armor.ypd_in_world[2]};
    a["ypd_world_deg"] = {armor.ypd_in_world[0] * 57.3, armor.ypd_in_world[1] * 57.3};

    if (std::isnan(armor.yaw_raw)) {
      a["yaw_raw_deg"] = nullptr;
    } else {
      a["yaw_raw_deg"] = armor.yaw_raw * 57.3;
    }

    a["center_px"] = {armor.center.x, armor.center.y};
    a["center_norm"] = {armor.center_norm.x, armor.center_norm.y};
    a["box_xywh"] = {armor.box.x, armor.box.y, armor.box.width, armor.box.height};

    a["points_px"] = nlohmann::json::array();
    for (const auto & p : armor.points) {
      a["points_px"].push_back({p.x, p.y});
    }
    data["armors"].push_back(std::move(a));
  }

  if (!armors.empty()) {
    const auto & armor = armors.front();
    // 保留历史单目标字段，兼容旧版 plotter 面板。
    data["armor_name"] = auto_aim::ARMOR_NAMES[static_cast<int>(armor.name)];
    data["armor_type"] = auto_aim::ARMOR_TYPES[static_cast<int>(armor.type)];
    data["armor_x"] = armor.xyz_in_world[0];
    data["armor_y"] = armor.xyz_in_world[1];
    data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    if (std::isnan(armor.yaw_raw)) {
      data["armor_yaw_raw"] = nullptr;
    } else {
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    }
    data["armor_center_x"] = armor.center_norm.x;
    data["armor_center_y"] = armor.center_norm.y;
  }
  data["gimbal_yaw"] = yaw * 57.3;
  data["cmd_yaw"] = command.yaw * 57.3;
  data["shoot"] = command.shoot;

  if (!targets.empty()) {
    const auto & target = targets.front();
    const std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
    for (const Eigen::Vector4d & xyza : armor_xyza_list) {
      auto image_points =
        solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 255, 0});
    }
    Eigen::VectorXd x = target.ekf_x();
    std::vector<cv::Point3f> center_pt = {
      {static_cast<float>(x[0]), static_cast<float>(x[2]), static_cast<float>(x[4])}};
    auto center_img_pts = solver.world2pixel(center_pt);
    if (!center_img_pts.empty()) {
      cv::circle(img, center_img_pts[0], 5, {255, 255, 0}, -1);
    }
    auto aim_point = aimer.debug_aim_point;
    Eigen::Vector4d aim_xyza = aim_point.xyza;
    auto aim_pts =
      solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
    if (aim_point.valid) {
      tools::draw_points(img, aim_pts, {0, 0, 255});
    }

    data["x"] = x[0];
    data["vx"] = x[1];
    data["y"] = x[2];
    data["vy"] = x[3];
    data["z"] = x[4];
    data["vz"] = x[5];
    data["a"] = x[6] * 57.3;
    data["w"] = x[7];
    data["r"] = x[8];
    data["l"] = x[9];
    data["h"] = x[10];
    data["last_id"] = target.last_id;

    auto ekf = target.ekf();
    data["residual_yaw"] = ekf.data.at("residual_yaw");
    data["residual_pitch"] = ekf.data.at("residual_pitch");
    data["residual_distance"] = ekf.data.at("residual_distance");
    data["residual_angle"] = ekf.data.at("residual_angle");
    data["nis"] = ekf.data.at("nis");
    data["nees"] = ekf.data.at("nees");
    data["nis_fail"] = ekf.data.at("nis_fail");
    data["nees_fail"] = ekf.data.at("nees_fail");
    data["recent_nis_failures"] = ekf.data.at("recent_nis_failures");
  }

  if (cfg.plotter && plotter != nullptr) {
    plotter->plot(data);
  }

  if (cfg.window) {
    if (cfg.resize_scale > 0.0 && std::abs(cfg.resize_scale - 1.0) > 1e-6) {
      cv::resize(img, img, {}, cfg.resize_scale, cfg.resize_scale);
    }
    cv::imshow("reprojection", img);
    const int key = cv::waitKey(1);
    return key == 'q' || key == 'Q';
  }
  return false;
}

/**
 * PnP 自检：绿/黄框由 cfg.pnp_viz_green / cfg.pnp_viz_yellow 控制；白字标注序号、名称、大小、距离
 * @return 同上，按 q 退出
 */
inline bool hero_viz_pnp_armors_frame(
  cv::Mat & img,
  auto_aim::Solver & solver,
  const std::list<auto_aim::Armor> & armors_solved,
  const HeroVizConfig & cfg)
{
  if (!cfg.window && !(cfg.plotter)) {
    return false;
  }

  if (armors_solved.empty() && cfg.window) {
    tools::draw_text(img, "no armor (PnP)", {10, 30}, {0, 0, 255});
  } else if (cfg.window) {
    const char * leg = "green: detect  yellow: PnP reproj";
    if (cfg.pnp_viz_green && !cfg.pnp_viz_yellow) {
      leg = "green: detect only (pnp_yellow=0)";
    } else if (!cfg.pnp_viz_green && cfg.pnp_viz_yellow) {
      leg = "yellow: PnP reproj only (pnp_green=0)";
    } else if (!cfg.pnp_viz_green && !cfg.pnp_viz_yellow) {
      leg = "pnp_green=0 pnp_yellow=0 (label only)";
    }
    tools::draw_text(img, leg, {10, 28}, {220, 220, 220}, 0.55, 2);
  }

  int i = 0;
  for (const auto & a : armors_solved) {
    const std::vector<cv::Point2f> pts =
      solver.reproject_armor(a.xyz_in_world, a.ypr_in_world[0], a.type, a.name);

    if (cfg.pnp_viz_green && a.points.size() == 4) {
      tools::draw_points(img, a.points, {0, 255, 0});
    }
    if (cfg.pnp_viz_yellow && pts.size() == 4) {
      tools::draw_points(img, pts, {0, 255, 255});
    }

    cv::Point2f c{0.F, 0.F};
    if (cfg.pnp_viz_yellow && pts.size() == 4) {
      for (const auto & p : pts) {
        c.x += p.x;
        c.y += p.y;
      }
      c.x /= static_cast<float>(pts.size());
      c.y /= static_cast<float>(pts.size());
    } else if (cfg.pnp_viz_green && a.points.size() == 4) {
      for (const auto & p : a.points) {
        c.x += p.x;
        c.y += p.y;
      }
      c.x /= static_cast<float>(a.points.size());
      c.y /= static_cast<float>(a.points.size());
    } else if (pts.size() == 4) {
      for (const auto & p : pts) {
        c.x += p.x;
        c.y += p.y;
      }
      c.x /= static_cast<float>(pts.size());
      c.y /= static_cast<float>(pts.size());
    }
    const double d = a.xyz_in_gimbal.norm();
    const char * nm = auto_aim::ARMOR_NAMES[static_cast<int>(a.name)].c_str();
    const char * tp = auto_aim::ARMOR_TYPES[static_cast<int>(a.type)].c_str();
    const int tx = std::max(4, static_cast<int>(c.x) - 40);
    const int ty = std::max(18, static_cast<int>(c.y) - 12);
    tools::draw_text(
      img,
      fmt::format("#{} {} {} {:.2f}m", i, nm, tp, d),
      {tx, ty},
      {255, 255, 255},
      0.55,
      2);
    ++i;
  }

  if (cfg.window) {
    if (cfg.resize_scale > 0.0 && std::abs(cfg.resize_scale - 1.0) > 1e-6) {
      cv::resize(img, img, {}, cfg.resize_scale, cfg.resize_scale);
    }
    cv::imshow("hero_pnp", img);
    const int key = cv::waitKey(1);
    return key == 'q' || key == 'Q';
  }
  return false;
}

} // namespace tools

#endif // TOOLS__HERO_AUTO_AIM_VIZ_HPP
