#ifndef TOOLS__HERO_ARMOR_DETECT_HPP
#define TOOLS__HERO_ARMOR_DETECT_HPP

#include <algorithm>
#include <list>
#include <limits>
#include <utility>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/phoenix_tradition_detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/logger.hpp"

namespace tools
{

/** 与 hero_detectwithpnp_test 一致：从 yaml 读取装甲检测相关开关 */
struct HeroArmorDetectYamlFlags
{
  bool use_phoenix_traditional{false};
  bool debug_img{false};
  bool phoenix_fuse_yolo_on_name_conf_reject{false};
  double phoenix_fuse_match_center_max_px{60.0};
  double phoenix_fuse_min_yolo_confidence{0.30};
  double phoenix_fuse_corner_mean_max_px{28.0};
  double phoenix_fuse_corner_max_px{55.0};
};

inline HeroArmorDetectYamlFlags hero_load_armor_detect_yaml_flags(const std::string & config_path)
{
  HeroArmorDetectYamlFlags f;
  try {
    const auto yaml = YAML::LoadFile(config_path);
    if (yaml["use_phoenix_traditional"].IsDefined()) {
      f.use_phoenix_traditional = yaml["use_phoenix_traditional"].as<bool>();
    }
    if (yaml["debug_img"].IsDefined()) {
      f.debug_img = yaml["debug_img"].as<bool>();
    }
    if (yaml["phoenix_tradition_detect"]) {
      const auto sub = yaml["phoenix_tradition_detect"];
      if (sub["fuse_yolo_on_name_conf_reject"]) {
        f.phoenix_fuse_yolo_on_name_conf_reject =
          sub["fuse_yolo_on_name_conf_reject"].as<bool>();
      }
      if (sub["fuse_match_center_max_px"]) {
        f.phoenix_fuse_match_center_max_px =
          std::max(1.0, sub["fuse_match_center_max_px"].as<double>());
      }
      if (sub["fuse_min_yolo_confidence"]) {
        f.phoenix_fuse_min_yolo_confidence =
          std::max(0.0, std::min(1.0, sub["fuse_min_yolo_confidence"].as<double>()));
      }
      if (sub["fuse_corner_mean_max_px"]) {
        f.phoenix_fuse_corner_mean_max_px =
          std::max(1.0, sub["fuse_corner_mean_max_px"].as<double>());
      }
      if (sub["fuse_corner_max_px"]) {
        f.phoenix_fuse_corner_max_px =
          std::max(f.phoenix_fuse_corner_mean_max_px, sub["fuse_corner_max_px"].as<double>());
      }
    }
  } catch (...) {
  }
  return f;
}

inline cv::Rect2f hero_armor_rect_from_points(const std::vector<cv::Point2f> & points)
{
  if (points.empty()) {
    return {};
  }
  float min_x = points[0].x;
  float min_y = points[0].y;
  float max_x = points[0].x;
  float max_y = points[0].y;
  for (const auto & p : points) {
    min_x = std::min(min_x, p.x);
    min_y = std::min(min_y, p.y);
    max_x = std::max(max_x, p.x);
    max_y = std::max(max_y, p.y);
  }
  const float w = std::max(0.0F, max_x - min_x);
  const float h = std::max(0.0F, max_y - min_y);
  return {min_x, min_y, w, h};
}

inline double hero_rect_iou(const cv::Rect2f & a, const cv::Rect2f & b)
{
  const auto inter = a & b;
  const double inter_area = static_cast<double>(inter.area());
  const double union_area =
    static_cast<double>(a.area()) + static_cast<double>(b.area()) - inter_area;
  if (union_area <= 1e-6) {
    return 0.0;
  }
  return inter_area / union_area;
}

inline std::pair<double, double> hero_corner_diff_mean_and_max(
  const std::vector<cv::Point2f> & a, const std::vector<cv::Point2f> & b)
{
  if (a.size() != 4 || b.size() != 4) {
    return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  }
  double best_mean = std::numeric_limits<double>::infinity();
  double best_max = std::numeric_limits<double>::infinity();
  for (int reverse = 0; reverse < 2; ++reverse) {
    for (int shift = 0; shift < 4; ++shift) {
      double sum_dist = 0.0;
      double max_dist = 0.0;
      for (int i = 0; i < 4; ++i) {
        const int j = reverse ? ((shift - i + 4) % 4) : ((i + shift) % 4);
        const double d = cv::norm(a[i] - b[j]);
        sum_dist += d;
        max_dist = std::max(max_dist, d);
      }
      const double mean_dist = sum_dist / 4.0;
      if (mean_dist < best_mean) {
        best_mean = mean_dist;
        best_max = max_dist;
      }
    }
  }
  return {best_mean, best_max};
}

/** debug_img 为 true 且走 Phoenix 或 CLI 传统分支时，为 Detector / Phoenix 打开二值化类调试图 */
inline bool hero_armor_tradition_visual_debug(
  bool debug_img, bool use_phoenix_traditional, bool tradition_cli)
{
  return debug_img && (use_phoenix_traditional || tradition_cli);
}

/** PhoenixTraditionDetector 构造第二参：仅 Phoenix 路径且 tradition_visual 时开启内部 debug */
inline bool hero_phoenix_detector_impl_debug(
  bool tradition_visual_debug, bool use_phoenix_traditional)
{
  return tradition_visual_debug && use_phoenix_traditional;
}

/**
 * 检测路径优先级：use_phoenix_traditional（yaml）> tradition_cli（--tradition）> YOLO。
 * frame_count 传给各 detect（与主程序帧号对齐；不关心时可传 -1）。
 */
inline std::list<auto_aim::Armor> hero_detect_armors_for_frame(
  const cv::Mat & img,
  int frame_count,
  const HeroArmorDetectYamlFlags & cfg,
  bool use_phoenix_traditional,
  bool tradition_cli,
  auto_aim::PhoenixTraditionDetector & phoenix,
  auto_aim::Detector & detector,
  auto_aim::YOLO & yolo)
{
  if (use_phoenix_traditional) {
    if (!cfg.phoenix_fuse_yolo_on_name_conf_reject) {
      return phoenix.detect(img, frame_count);
    }

    std::vector<auto_aim::Armor> name_conf_rejected_armors;
    auto armors = phoenix.detect(img, frame_count, &name_conf_rejected_armors);
    if (name_conf_rejected_armors.empty()) {
      return armors;
    }

    auto yolo_armors = yolo.detect(img, frame_count);
    if (yolo_armors.empty()) {
      tools::logger()->warn(
        "[detect][fusion] phoenix name/conf reject={} but yolo armors=0, no fallback",
        name_conf_rejected_armors.size());
      return armors;
    }

    int fused_count = 0;
    for (auto & cand : name_conf_rejected_armors) {
      const auto cand_rect = hero_armor_rect_from_points(cand.points);
      double best_score = -std::numeric_limits<double>::infinity();
      std::list<auto_aim::Armor>::const_iterator best = yolo_armors.end();

      for (auto it = yolo_armors.begin(); it != yolo_armors.end(); ++it) {
        const auto & y = *it;
        if (y.confidence < cfg.phoenix_fuse_min_yolo_confidence) {
          continue;
        }
        if (y.color != cand.color) {
          continue;
        }
        const double center_dist = cv::norm(y.center - cand.center);
        if (center_dist > cfg.phoenix_fuse_match_center_max_px) {
          continue;
        }
        const double iou = hero_rect_iou(cand_rect, hero_armor_rect_from_points(y.points));
        if (iou < 0.02) {
          continue;
        }
        const double score = iou - 0.002 * center_dist + 0.05 * y.confidence;
        if (score > best_score) {
          best_score = score;
          best = it;
        }
      }

      if (best == yolo_armors.end()) {
        continue;
      }
      const auto [corner_mean, corner_max] =
        hero_corner_diff_mean_and_max(cand.points, best->points);
      if (
        corner_mean > cfg.phoenix_fuse_corner_mean_max_px ||
        corner_max > cfg.phoenix_fuse_corner_max_px) {
        tools::logger()->warn(
          "[detect][fusion] reject by corner-check mean={:.1f}px max={:.1f}px thr_mean={:.1f}px thr_max={:.1f}px",
          corner_mean,
          corner_max,
          cfg.phoenix_fuse_corner_mean_max_px,
          cfg.phoenix_fuse_corner_max_px);
        continue;
      }

      cand.name = best->name;
      cand.confidence = best->confidence;
      cand.type = best->type;
      cand.class_id = best->class_id;
      cand.center_norm = {
        cand.center.x / std::max(1, img.cols), cand.center.y / std::max(1, img.rows)};
      armors.push_back(cand);
      ++fused_count;
    }

    if (fused_count > 0) {
      tools::logger()->warn(
        "[detect][fusion] phoenix_reject={} fused_by_yolo={} total_after_fuse={}",
        name_conf_rejected_armors.size(),
        fused_count,
        armors.size());
    }
    return armors;
  }
  if (tradition_cli) {
    return detector.detect(img, frame_count);
  }
  return yolo.detect(img, frame_count);
}

inline bool hero_armor_detect_using_yolo_path(
  [[maybe_unused]] const HeroArmorDetectYamlFlags & cfg, bool use_phoenix_traditional, bool tradition_cli)
{
  return !use_phoenix_traditional && !tradition_cli;
}

} // namespace tools

#endif // TOOLS__HERO_ARMOR_DETECT_HPP
