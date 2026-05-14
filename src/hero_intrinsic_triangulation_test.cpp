/**
 * @brief 形似 hero_detectwithpnp_test：ROS 读图 + 检测，额外做「仅内参」的简易三角测距并与 PnP 对比。
 *
 * 模型：针孔相似三角形。水平方向 Z_w ≈ fx * W / w_px，竖直方向 Z_h ≈ fy * H / h_px，
 * 其中 W/H 与 Solver 中 PnP 板模型一致（大/小装甲宽度 + 灯条方向高度 56mm），
 * w_px/h_px 为图像上边长与侧边长的像素均值。目标近似正对相机时二者应接近，取平均作为简易深度。
 *
 * 用法：
 *   ./build/hero_intrinsic_triangulation_test configs/hero.yaml
 *   ./build/hero_intrinsic_triangulation_test configs/hero.yaml --tradition=true --ros_topic=/image_for_auto_aim
 * 检测路径与 hero_detectwithpnp_test 一致：yaml use_phoenix_traditional > --tradition > YOLO。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <list>
#include <string>

#include <Eigen/Geometry>
#include <fmt/format.h>
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

namespace
{
/** 与 tasks/auto_aim/solver.cpp 中 PnP 板尺寸一致，便于对比 */
constexpr double kLightbarLengthM = 56e-3;
constexpr double kBigArmorWidthM = 230e-3;
constexpr double kSmallArmorWidthM = 135e-3;

struct CamIntrinsics
{
  double fx{0};
  double fy{0};
  bool ok{false};
};

CamIntrinsics load_intrinsics(const std::string & config_path)
{
  CamIntrinsics out;
  try {
    auto yaml = YAML::LoadFile(config_path);
    auto m = yaml["camera_matrix"].as<std::vector<double>>();
    if (m.size() >= 9) {
      out.fx = m[0];
      out.fy = m[4];
      out.ok = out.fx > 1.0 && out.fy > 1.0;
    }
  } catch (const std::exception & e) {
    tools::logger()->error("读取 camera_matrix 失败: {}", e.what());
  }
  return out;
}

struct TriDepthResult
{
  double z_w{NAN};   // 按板宽 W
  double z_h{NAN};   // 按灯条跨度 H
  double z_avg{NAN}; // 二者平均
};

TriDepthResult tri_depth_from_corners(
  const CamIntrinsics & cam, const auto_aim::Armor & a)
{
  TriDepthResult r;
  if (!cam.ok || a.points.size() != 4) {
    return r;
  }
  const auto & p = a.points;
  const double w_px =
    0.5 * (cv::norm(p[1] - p[0]) + cv::norm(p[2] - p[3]));
  const double h_px =
    0.5 * (cv::norm(p[3] - p[0]) + cv::norm(p[2] - p[1]));
  if (w_px < 1.0 || h_px < 1.0) {
    return r;
  }
  const double W =
    (a.type == auto_aim::ArmorType::big) ? kBigArmorWidthM : kSmallArmorWidthM;
  const double H = kLightbarLengthM;
  r.z_w = cam.fx * W / w_px;
  r.z_h = cam.fy * H / h_px;
  if (std::isfinite(r.z_w) && std::isfinite(r.z_h)) {
    r.z_avg = 0.5 * (r.z_w + r.z_h);
  }
  return r;
}

static const std::string kCliKeys =
    "{help h usage ? | | 输出帮助}"
    "{@config-path | configs/hero.yaml | yaml 配置文件路径}"
    "{tradition t | false | 是否使用传统方法识别（Sc_vision Detector；Phoenix 开启时无效）}"
    "{ros_topic | /image_for_auto_aim | ROS 图像话题}"
    "{queue q | 3 | 订阅队列深度}"
    "{csv | | 追加写入 CSV（PnP 距离与三角测距对比）}";

static void write_csv_header(std::ofstream & out)
{
  out << "timestamp_ns,frame,armor_idx,armor_type,armor_name,"
         "pnp_dist_m,tri_z_w_m,tri_z_h_m,tri_z_avg_m\n";
}

} // namespace

int main(int argc, char ** argv)
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
  const std::string csv_path = cli.get<std::string>("csv");

  const auto armor_yaml = tools::hero_load_armor_detect_yaml_flags(config_path);
  const bool use_phoenix_traditional = armor_yaml.use_phoenix_traditional;
  const bool tradition_visual = tools::hero_armor_tradition_visual_debug(
      armor_yaml.debug_img, use_phoenix_traditional, use_tradition);

  std::ofstream csv_out;
  if (!csv_path.empty()) {
    namespace fs = std::filesystem;
    const bool need_header = !fs::exists(csv_path) || fs::file_size(csv_path) == 0;
    csv_out.open(csv_path, std::ios::out | std::ios::app);
    if (!csv_out) {
      tools::logger()->error("无法打开 CSV: {}", csv_path);
      return 1;
    }
    if (need_header) {
      write_csv_header(csv_out);
    }
    tools::logger()->info("CSV: {}", csv_path);
  }

  rclcpp::init(argc, argv);

  const CamIntrinsics intr = load_intrinsics(config_path);
  if (!intr.ok) {
    tools::logger()->error("内参无效，请检查 yaml 中 camera_matrix");
    rclcpp::shutdown();
    return 1;
  }
  tools::logger()->info("内参 fx={:.3f} fy={:.3f}（用于三角测距）", intr.fx, intr.fy);

  tools::logger()->info(
      "hero_intrinsic_triangulation: use_phoenix_traditional={} tradition_cli={} debug_img={}",
      use_phoenix_traditional,
      use_tradition,
      armor_yaml.debug_img);

  tools::Exiter exiter;
  io::ImageFromRos camera(image_topic, static_cast<std::size_t>(queue_cap), false);

  auto_aim::Detector detector(config_path, tradition_visual);
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::PhoenixTraditionDetector phoenix_detector(
      config_path,
      tools::hero_phoenix_detector_impl_debug(tradition_visual, use_phoenix_traditional));
  auto_aim::Solver solver(config_path);

  const Eigen::Quaterniond identity_q = Eigen::Quaterniond::Identity();

  std::chrono::steady_clock::time_point timestamp;
  int total_frames = 0;
  int detect_frames = 0;

  while (!exiter.exit() && rclcpp::ok()) {
    cv::Mat img;
    std::list<auto_aim::Armor> armors;

    camera.read(img, timestamp);
    if (img.empty()) {
      break;
    }

    auto last = std::chrono::steady_clock::now();

    armors = tools::hero_detect_armors_for_frame(
        img,
        total_frames,
        armor_yaml,
        use_phoenix_traditional,
        use_tradition,
        phoenix_detector,
        detector,
        yolo);
    if (tools::hero_armor_detect_using_yolo_path(use_phoenix_traditional, use_tradition)) {
      tools::hero_log_yolo_roi(yolo);
    }

    solver.set_R_gimbal2world(identity_q);

    cv::Mat viz = img.clone();
    if (!armors.empty()) {
      int armor_idx = 0;
      for (auto & a : armors) {
        solver.solve(a);
        const TriDepthResult tri = tri_depth_from_corners(intr, a);
        const double pnp_d = a.xyz_in_gimbal.norm();

        if (csv_out.is_open()) {
          const int64_t t_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   timestamp.time_since_epoch())
                                   .count();
          csv_out << fmt::format(
            "{},{},{},{},{},{:.6f},{:.6f},{:.6f},{:.6f}\n", t_ns, total_frames, armor_idx,
            auto_aim::ARMOR_TYPES[static_cast<int>(a.type)],
            auto_aim::ARMOR_NAMES[static_cast<int>(a.name)], pnp_d, tri.z_w, tri.z_h, tri.z_avg);
          ++armor_idx;
        }

        if (a.points.size() == 4) {
          tools::draw_points(viz, a.points, {0, 255, 0});
          cv::Point2f c{0.F, 0.F};
          for (const auto & pt : a.points) {
            c.x += pt.x;
            c.y += pt.y;
          }
          c.x /= 4.F;
          c.y /= 4.F;
          const int tx = std::max(4, static_cast<int>(c.x) - 80);
          const int ty = std::max(16, static_cast<int>(c.y) - 10);
          tools::draw_text(
            viz,
            fmt::format(
              "{} PnP:{:.2f}m Tri:{:.2f}m", auto_aim::ARMOR_NAMES[static_cast<int>(a.name)].c_str(),
              pnp_d, tri.z_avg),
            {tx, ty}, {255, 255, 255}, 0.5, 2);
          if (std::isfinite(tri.z_w) && std::isfinite(tri.z_h)) {
            tools::draw_text(
              viz, fmt::format("(Zw:{:.2f} Zh:{:.2f})", tri.z_w, tri.z_h), {tx, ty + 16},
              {200, 200, 200}, 0.45, 1);
          }
        }
      }
    } else {
      tools::draw_text(viz, "no armor", {10, 30}, {0, 0, 255}, 0.7, 2);
    }

    tools::draw_text(
      viz, "Green: corners | PnP vs intrinsic triangulation (Zavg)", {10, 24}, {0, 255, 255}, 0.55,
      2);

    cv::imshow("intrinsic_triangulation", viz);

    total_frames++;
    if (!armors.empty()) {
      detect_frames++;
    }

    const double detect_rate = total_frames > 0 ? 100.0 * detect_frames / total_frames : 0.0;
    const auto now = std::chrono::steady_clock::now();
    const double dt = tools::delta_time(now, last);
    tools::logger()->info(
      "{:.2f} fps  detect_rate={:.1f}%  ({}/{})", 1.0 / std::max(dt, 1e-6), detect_rate,
      detect_frames, total_frames);

    const int key = cv::waitKey(33);
    if (key == 'q' || key == 'Q') {
      break;
    }
  }

  rclcpp::shutdown();
  return 0;
}
