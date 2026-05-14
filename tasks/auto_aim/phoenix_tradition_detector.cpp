#include "phoenix_tradition_detector.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
namespace
{
// 与 armor_detector/include/armor_detector/armor.hpp 一致，供 AdjustTopBottom 使用
constexpr float kSmallArmorHeight = 0.0485F;
constexpr float kSmallArmorLongHeight = 0.055F;

/** 与 armor::Light 等价的几何（不继承 RotatedRect，避免与 OpenCV 类型纠缠） */
struct PhoenixLight
{
  cv::Point2f point[4];
  cv::Point2f top, bottom;
  cv::Point2f adjusted_top, adjusted_bottom;
  cv::Point2f center;
  float length = 0.F, width = 0.F, ratio = 0.F, tilt_angle = 0.F;
  bool valid = false;

  explicit PhoenixLight(const cv::RotatedRect & light_box)
  {
    light_box.points(point);
    center = light_box.center;
    std::sort(point, point + 4, [this](const cv::Point2f & a, const cv::Point2f & b) {
      return std::atan2(a.y - center.y, a.x - center.x) >
             std::atan2(b.y - center.y, b.x - center.x);
    });

    top = (point[0] + point[1]) / 2.F;
    bottom = (point[2] + point[3]) / 2.F;
    length = static_cast<float>(cv::norm(top - bottom));
    width = static_cast<float>(cv::norm(point[0] - point[1]));
    ratio = length / std::max(width, 1e-6F);
    tilt_angle = static_cast<float>(std::atan((top.x - bottom.x) / (top.y - bottom.y + 1e-6F)) / CV_PI * 180.0);
    adjusted_top = top;
    adjusted_bottom = bottom;
  }
};

struct PhoenixPairMetrics
{
  float light_height_ratio = 0.F;
  float light_angle_diff = 0.F;
  float angle = 0.F;
  float light_center_distance = 0.F;
};

struct PhoenixGeometryConfig
{
  double light_height_ratio_min = 0.8;
  double light_height_ratio_max = 1.2;
  double light_angle_diff_max_deg = 10.0;
  double pair_angle_max_deg = 30.0;
  double center_distance_min_1 = 0.8;
  double center_distance_max_1 = 3.2;
  double center_distance_min_2 = 3.2;
  double center_distance_max_2 = 5.5;
};

struct PhoenixRejectInfo
{
  bool height_ratio_ok = true;
  bool angle_diff_ok = true;
  bool pair_angle_ok = true;
  bool center_distance_ok = true;
};

PhoenixPairMetrics compute_metrics(const PhoenixLight & L, const PhoenixLight & R)
{
  PhoenixPairMetrics m;
  m.light_height_ratio = L.length / std::max(R.length, 1e-6F);
  m.light_angle_diff =
    std::abs(std::abs(L.tilt_angle) - std::abs(R.tilt_angle));
  cv::Point2f diff = L.center - R.center;
  m.angle = static_cast<float>(std::abs(std::atan(diff.y / (diff.x + 1e-6F))) / CV_PI * 180.0);
  float avg_len = (L.length + R.length) / 2.F;
  m.light_center_distance = static_cast<float>(cv::norm(L.center - R.center)) / std::max(avg_len, 1e-6F);
  return m;
}

std::string reject_reason(const PhoenixRejectInfo & info)
{
  std::ostringstream oss;
  bool first = true;
  auto append = [&](const char * tag) {
    if (!first) {
      oss << ",";
    }
    oss << tag;
    first = false;
  };
  if (!info.height_ratio_ok) {
    append("height_ratio");
  }
  if (!info.angle_diff_ok) {
    append("angle_diff");
  }
  if (!info.pair_angle_ok) {
    append("pair_angle");
  }
  if (!info.center_distance_ok) {
    append("center_distance");
  }
  return first ? std::string("none") : oss.str();
}

bool geometry_ok_for_pair(
  const PhoenixPairMetrics & m, const PhoenixGeometryConfig & cfg, PhoenixRejectInfo * info_out)
{
  PhoenixRejectInfo info;
  info.height_ratio_ok =
    m.light_height_ratio > cfg.light_height_ratio_min &&
    m.light_height_ratio < cfg.light_height_ratio_max;
  info.angle_diff_ok = m.light_angle_diff < cfg.light_angle_diff_max_deg;
  info.pair_angle_ok = m.angle < cfg.pair_angle_max_deg;
  info.center_distance_ok =
    (m.light_center_distance > cfg.center_distance_min_1 &&
     m.light_center_distance < cfg.center_distance_max_1) ||
    (m.light_center_distance > cfg.center_distance_min_2 &&
     m.light_center_distance < cfg.center_distance_max_2);
  if (info_out != nullptr) {
    *info_out = info;
  }
  return info.height_ratio_ok && info.angle_diff_ok && info.pair_angle_ok &&
         info.center_distance_ok;
}

void adjust_top_bottom(PhoenixLight & light)
{
  const double ratio = (1.0 - static_cast<double>(kSmallArmorHeight / kSmallArmorLongHeight)) * 0.5;
  cv::Point2f eigenvector = light.bottom - light.top;
  double len =
    std::sqrt(static_cast<double>(eigenvector.x * eigenvector.x + eigenvector.y * eigenvector.y));
  if (len < 1e-6) {
    return;
  }
  eigenvector.x = static_cast<float>(eigenvector.x / len);
  eigenvector.y = static_cast<float>(eigenvector.y / len);
  light.adjusted_top = light.top + eigenvector * static_cast<float>(light.length * ratio);
  light.adjusted_bottom = light.bottom - eigenvector * static_cast<float>(light.length * ratio);
}

Lightbar lightbar_from_axis(
  const cv::Point2f & top, const cv::Point2f & bottom, float width, std::size_t id, Color color)
{
  cv::Point2f axis = bottom - top;
  float len = static_cast<float>(cv::norm(axis));
  if (len < 1e-3F) {
    axis = {0.F, 1.F};
    len = 1.F;
  } else {
    axis /= len;
  }
  cv::Point2f perp{-axis.y, axis.x};
  const float hw = width * 0.5F;
  std::vector<cv::Point2f> c = {
    top + perp * hw, top - perp * hw, bottom - perp * hw, bottom + perp * hw};
  cv::RotatedRect rr = cv::minAreaRect(c);
  Lightbar lb(rr, id);
  lb.color = color;
  return lb;
}

cv::Mat get_pattern_bgr(const cv::Mat & bgr_img, const Armor & armor)
{
  const auto tl = armor.left.center - armor.left.top2bottom * 1.125F;
  const auto bl = armor.left.center + armor.left.top2bottom * 1.125F;
  const auto tr = armor.right.center - armor.right.top2bottom * 1.125F;
  const auto br = armor.right.center + armor.right.top2bottom * 1.125F;

  const int roi_left = std::max<int>(std::min(static_cast<int>(tl.x), static_cast<int>(bl.x)), 0);
  const int roi_top = std::max<int>(std::min(static_cast<int>(tl.y), static_cast<int>(tr.y)), 0);
  const int roi_right =
    std::min<int>(std::max(static_cast<int>(tr.x), static_cast<int>(br.x)), bgr_img.cols);
  const int roi_bottom =
    std::min<int>(std::max(static_cast<int>(bl.y), static_cast<int>(br.y)), bgr_img.rows);
  const cv::Point roi_tl(roi_left, roi_top);
  const cv::Point roi_br(roi_right, roi_bottom);
  const cv::Rect roi(roi_tl, roi_br);
  if (roi.width <= 0 || roi.height <= 0) {
    return {};
  }
  return bgr_img(roi).clone();
}

ArmorType get_armor_type(const Armor & armor, double big_threshold, double small_threshold)
{
  if (armor.ratio > big_threshold) {
    return ArmorType::big;
  }
  if (armor.ratio < small_threshold) {
    return ArmorType::small;
  }
  if (armor.name == ArmorName::one || armor.name == ArmorName::base) {
    return ArmorType::big;
  }
  return ArmorType::small;
}

bool check_name(const Armor & armor, double min_confidence)
{
  const bool name_ok = armor.name != ArmorName::not_armor;
  const bool confidence_ok = armor.confidence > min_confidence;
  return name_ok && confidence_ok;
}

bool check_type(const Armor & armor)
{
  const bool name_ok =
    armor.type == ArmorType::small
      ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
      : (armor.name == ArmorName::one || armor.name == ArmorName::base);
  if (!name_ok) {
    tools::logger()->debug(
      "[PhoenixTraditionDetector] strange armor: {} {}", ARMOR_TYPES[armor.type],
      ARMOR_NAMES[armor.name]);
  }
  return name_ok;
}

PhoenixLight form_light(
  const std::vector<cv::Point> & contour, double min_ratio, double max_tilt_angle_deg)
{
  PhoenixLight light(cv::minAreaRect(contour));
  light.valid = (light.length > light.width * min_ratio) &&
                (std::abs(light.tilt_angle) < max_tilt_angle_deg);
  return light;
}

bool contain_light(
  const PhoenixLight & light1, const PhoenixLight & light2, const std::vector<PhoenixLight> & lights)
{
  return std::any_of(lights.begin(), lights.end(), [&](const PhoenixLight & light) {
    return (light.center.y > light1.center.y && light.center.y < light2.center.y) &&
           (light.center.x > light1.center.x && light.center.x < light2.center.x);
  });
}

Armor make_candidate_armor(
  PhoenixLight left_light, PhoenixLight right_light, Color enemy_color)
{
  adjust_top_bottom(left_light);
  adjust_top_bottom(right_light);

  Lightbar llb =
    lightbar_from_axis(left_light.top, left_light.bottom, left_light.width, 0U, enemy_color);
  Lightbar rlb =
    lightbar_from_axis(right_light.top, right_light.bottom, right_light.width, 1U, enemy_color);

  Armor armor(llb, rlb);
  armor.points[0] = left_light.adjusted_top;
  armor.points[1] = right_light.adjusted_top;
  armor.points[2] = right_light.adjusted_bottom;
  armor.points[3] = left_light.adjusted_bottom;
  return armor;
}

}  // namespace

PhoenixTraditionDetector::PhoenixTraditionDetector(const std::string & config_path, bool debug)
: classifier_(config_path), debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);

  binary_threshold_[0] = 100;
  binary_threshold_[1] = 100;
  light_contour_threshold_[0] = 100;
  light_contour_threshold_[1] = 100;
  dilate_kernel_size_ = 5;
  min_contour_points_ = 10;
  light_min_ratio_ = 1.5;
  light_max_tilt_angle_deg_ = 25.0;
  pair_light_height_ratio_min_ = 0.8;
  pair_light_height_ratio_max_ = 1.2;
  pair_light_angle_diff_max_deg_ = 10.0;
  pair_center_line_angle_max_deg_ = 30.0;
  pair_center_distance_small_min_ = 0.8;
  pair_center_distance_small_max_ = 3.2;
  pair_center_distance_big_min_ = 3.2;
  pair_center_distance_big_max_ = 5.5;
  enable_contain_light_filter_ = true;
  viz_show_gray_binary_ = true;
  viz_show_light_binary_ = true;
  viz_show_light_boxes_ = true;
  viz_show_accepted_armors_ = true;
  viz_show_suspicious_armors_ = true;
  viz_show_suspicious_geometry_ = true;
  viz_show_suspicious_classify_ = true;
  viz_show_suspicious_name_conf_ = true;
  viz_show_suspicious_type_ = true;
  viz_show_suspicious_reason_text_ = true;
  viz_show_number_img_ = true;
  viz_show_number_img_geometry_reject_ = false;
  viz_show_number_img_name_conf_reject_ = true;
  viz_show_number_img_type_reject_ = true;
  suspicious_debug_ = false;
  suspicious_min_confidence_ = 0.0;
  armor_ratio_big_threshold_ = 3.0;
  armor_ratio_small_threshold_ = 2.5;

  if (yaml["phoenix_tradition_detect"]) {
    const auto sub = yaml["phoenix_tradition_detect"];
    if (sub["binary_threshold_red"]) {
      binary_threshold_[0] = sub["binary_threshold_red"].as<int>();
    }
    if (sub["binary_threshold_blue"]) {
      binary_threshold_[1] = sub["binary_threshold_blue"].as<int>();
    }
    if (sub["light_contour_threshold_red"]) {
      light_contour_threshold_[0] = sub["light_contour_threshold_red"].as<int>();
    }
    if (sub["light_contour_threshold_blue"]) {
      light_contour_threshold_[1] = sub["light_contour_threshold_blue"].as<int>();
    }
    if (sub["dilate_kernel_size"]) {
      dilate_kernel_size_ = std::max(1, sub["dilate_kernel_size"].as<int>());
    }
    if (sub["min_contour_points"]) {
      min_contour_points_ = std::max(1, sub["min_contour_points"].as<int>());
    }
    if (sub["light_min_ratio"]) {
      light_min_ratio_ = std::max(1.0, sub["light_min_ratio"].as<double>());
    }
    if (sub["light_max_tilt_angle_deg"]) {
      light_max_tilt_angle_deg_ = std::max(1.0, sub["light_max_tilt_angle_deg"].as<double>());
    }
    if (sub["pair_light_height_ratio_min"]) {
      pair_light_height_ratio_min_ = std::max(0.0, sub["pair_light_height_ratio_min"].as<double>());
    }
    if (sub["pair_light_height_ratio_max"]) {
      pair_light_height_ratio_max_ =
        std::max(pair_light_height_ratio_min_ + 1e-6, sub["pair_light_height_ratio_max"].as<double>());
    }
    if (sub["pair_light_angle_diff_max_deg"]) {
      pair_light_angle_diff_max_deg_ = std::max(0.0, sub["pair_light_angle_diff_max_deg"].as<double>());
    }
    if (sub["pair_center_line_angle_max_deg"]) {
      pair_center_line_angle_max_deg_ = std::max(0.0, sub["pair_center_line_angle_max_deg"].as<double>());
    }
    if (sub["pair_center_distance_small_min"]) {
      pair_center_distance_small_min_ =
        std::max(0.0, sub["pair_center_distance_small_min"].as<double>());
    }
    if (sub["pair_center_distance_small_max"]) {
      pair_center_distance_small_max_ = std::max(
        pair_center_distance_small_min_ + 1e-6, sub["pair_center_distance_small_max"].as<double>());
    }
    if (sub["pair_center_distance_big_min"]) {
      pair_center_distance_big_min_ =
        std::max(0.0, sub["pair_center_distance_big_min"].as<double>());
    }
    if (sub["pair_center_distance_big_max"]) {
      pair_center_distance_big_max_ = std::max(
        pair_center_distance_big_min_ + 1e-6, sub["pair_center_distance_big_max"].as<double>());
    }
    if (sub["enable_contain_light_filter"]) {
      enable_contain_light_filter_ = sub["enable_contain_light_filter"].as<bool>();
    }
    if (sub["viz_show_gray_binary"]) {
      viz_show_gray_binary_ = sub["viz_show_gray_binary"].as<bool>();
    }
    if (sub["viz_show_light_binary"]) {
      viz_show_light_binary_ = sub["viz_show_light_binary"].as<bool>();
    }
    if (sub["viz_show_light_boxes"]) {
      viz_show_light_boxes_ = sub["viz_show_light_boxes"].as<bool>();
    }
    if (sub["viz_show_accepted_armors"]) {
      viz_show_accepted_armors_ = sub["viz_show_accepted_armors"].as<bool>();
    }
    if (sub["viz_show_suspicious_armors"]) {
      viz_show_suspicious_armors_ = sub["viz_show_suspicious_armors"].as<bool>();
    }
    if (sub["viz_show_suspicious_geometry"]) {
      viz_show_suspicious_geometry_ = sub["viz_show_suspicious_geometry"].as<bool>();
    }
    if (sub["viz_show_suspicious_classify"]) {
      viz_show_suspicious_classify_ = sub["viz_show_suspicious_classify"].as<bool>();
    }
    if (sub["viz_show_suspicious_name_conf"]) {
      viz_show_suspicious_name_conf_ = sub["viz_show_suspicious_name_conf"].as<bool>();
    }
    if (sub["viz_show_suspicious_type"]) {
      viz_show_suspicious_type_ = sub["viz_show_suspicious_type"].as<bool>();
    }
    if (sub["viz_show_suspicious_reason_text"]) {
      viz_show_suspicious_reason_text_ = sub["viz_show_suspicious_reason_text"].as<bool>();
    }
    if (sub["viz_show_number_img"]) {
      viz_show_number_img_ = sub["viz_show_number_img"].as<bool>();
    }
    if (sub["viz_show_number_img_geometry_reject"]) {
      viz_show_number_img_geometry_reject_ =
        sub["viz_show_number_img_geometry_reject"].as<bool>();
    }
    if (sub["viz_show_number_img_name_conf_reject"]) {
      viz_show_number_img_name_conf_reject_ =
        sub["viz_show_number_img_name_conf_reject"].as<bool>();
    }
    if (sub["viz_show_number_img_type_reject"]) {
      viz_show_number_img_type_reject_ =
        sub["viz_show_number_img_type_reject"].as<bool>();
    }
    if (sub["suspicious_debug"]) {
      suspicious_debug_ = sub["suspicious_debug"].as<bool>();
    }
    if (sub["suspicious_min_confidence"]) {
      suspicious_min_confidence_ = std::max(0.0, sub["suspicious_min_confidence"].as<double>());
    }
    if (sub["armor_ratio_big_threshold"]) {
      armor_ratio_big_threshold_ = std::max(1.0, sub["armor_ratio_big_threshold"].as<double>());
    }
    if (sub["armor_ratio_small_threshold"]) {
      armor_ratio_small_threshold_ = std::max(0.1, sub["armor_ratio_small_threshold"].as<double>());
    }
  }

  std::string ec = "blue";
  if (yaml["enemy_color"]) {
    ec = yaml["enemy_color"].as<std::string>();
  }
  enemy_is_red_ = (ec == "red" || ec == "Red" || ec == "RED");

  min_confidence_ = yaml["min_confidence"] ? yaml["min_confidence"].as<double>() : 0.7;

  if (yaml["debug_img"].IsDefined()) {
    debug_img_ = yaml["debug_img"].as<bool>();
  }

  const int k = dilate_kernel_size_;
  kernel_ = cv::Mat::ones(k, k, CV_8U);
}

cv::Mat PhoenixTraditionDetector::preprocess_image(const cv::Mat & input) const
{
  cv::Mat gray;
  cv::Mat binary;
  cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
  const int idx = enemy_is_red_ ? 0 : 1;
  cv::threshold(gray, binary, binary_threshold_[idx], 255, cv::THRESH_BINARY);
  return binary;
}

std::list<Armor> PhoenixTraditionDetector::detect(
  const cv::Mat & bgr_img, int frame_count, std::vector<Armor> * name_conf_rejected_armors)
{
  (void)frame_count;
  std::list<Armor> armors;
  if (bgr_img.empty()) {
    return armors;
  }

  preprocessed_image_ = preprocess_image(bgr_img);

  cv::split(bgr_img, channels_);
  if (enemy_is_red_) {
    cv::subtract(channels_[2], channels_[0], color_mask_);
  } else {
    cv::subtract(channels_[0], channels_[2], color_mask_);
  }
  const int idx = enemy_is_red_ ? 0 : 1;
  cv::threshold(
    color_mask_, light_contour_binary_image_, light_contour_threshold_[idx], 255, cv::THRESH_BINARY);
  cv::dilate(
    light_contour_binary_image_, light_contour_binary_image_, kernel_, cv::Point(-1, -1), 1);
  cv::bitwise_and(preprocessed_image_, light_contour_binary_image_, light_contour_binary_image_);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(
    light_contour_binary_image_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  struct SuspiciousArmorDebug
  {
    Armor armor;
    std::string reason;
    PhoenixPairMetrics metrics;
    bool contain_light_rejected{false};
    float left_tilt{0.F};
    float right_tilt{0.F};
  };

  std::vector<PhoenixLight> lights;
  std::vector<SuspiciousArmorDebug> suspicious_armors;
  cv::Mat latest_number_img;
  std::string latest_number_img_tag;
  PhoenixGeometryConfig pair_cfg;
  pair_cfg.light_height_ratio_min = pair_light_height_ratio_min_;
  pair_cfg.light_height_ratio_max = pair_light_height_ratio_max_;
  pair_cfg.light_angle_diff_max_deg = pair_light_angle_diff_max_deg_;
  pair_cfg.pair_angle_max_deg = pair_center_line_angle_max_deg_;
  pair_cfg.center_distance_min_1 = pair_center_distance_small_min_;
  pair_cfg.center_distance_max_1 = pair_center_distance_small_max_;
  pair_cfg.center_distance_min_2 = pair_center_distance_big_min_;
  pair_cfg.center_distance_max_2 = pair_center_distance_big_max_;

  for (const auto & contour : contours) {
    if (static_cast<int>(contour.size()) < min_contour_points_) {
      continue;
    }
    PhoenixLight light = form_light(contour, light_min_ratio_, light_max_tilt_angle_deg_);
    if (light.valid) {
      lights.push_back(light);
    }
  }

  const Color enemy_color = enemy_is_red_ ? Color::red : Color::blue;

  for (const auto & left_light : lights) {
    for (const auto & right_light : lights) {
      if (left_light.center.x >= right_light.center.x) {
        continue;
      }
      const PhoenixPairMetrics m = compute_metrics(left_light, right_light);
      PhoenixRejectInfo reject_info;
      const bool geometry_ok = geometry_ok_for_pair(m, pair_cfg, &reject_info);
      const bool contain_rejected =
        enable_contain_light_filter_ && contain_light(left_light, right_light, lights);

      if (!geometry_ok || contain_rejected) {
        if (suspicious_debug_) {
          Armor suspicious = make_candidate_armor(left_light, right_light, enemy_color);
          suspicious.pattern = get_pattern_bgr(bgr_img, suspicious);
          if (viz_show_number_img_ && viz_show_number_img_geometry_reject_) {
            latest_number_img = suspicious.pattern.clone();
            latest_number_img_tag = "sus_geometry";
          }
          classifier_.classify(suspicious);
          if (suspicious.confidence >= suspicious_min_confidence_) {
            std::string reason = geometry_ok ? std::string("contain_light")
                                             : reject_reason(reject_info);
            if (contain_rejected && !reason.empty() && reason != "none") {
              reason += ",contain_light";
            }
            suspicious_armors.push_back(
              {suspicious,
               reason,
               m,
               contain_rejected,
               left_light.tilt_angle,
               right_light.tilt_angle});
            tools::logger()->warn(
              "[PhoenixTraditionDetector][suspicious] reject={} contain_light={} "
              "name={} conf={:.2f} h_ratio={:.2f} angle_diff={:.2f} pair_angle={:.2f} "
              "center_dist={:.2f} tiltL={:.2f} tiltR={:.2f}",
              reason.c_str(),
              contain_rejected,
              ARMOR_NAMES[static_cast<int>(suspicious.name)],
              suspicious.confidence,
              m.light_height_ratio,
              m.light_angle_diff,
              m.angle,
              m.light_center_distance,
              left_light.tilt_angle,
              right_light.tilt_angle);
          }
        }
        continue;
      }

      Armor armor = make_candidate_armor(left_light, right_light, enemy_color);
      armor.pattern = get_pattern_bgr(bgr_img, armor);
      classifier_.classify(armor);
      const bool name_ok = check_name(armor, min_confidence_);
      if (!name_ok) {
        if (name_conf_rejected_armors != nullptr) {
          name_conf_rejected_armors->push_back(armor);
        }
        if (suspicious_debug_ && armor.confidence >= suspicious_min_confidence_) {
          suspicious_armors.push_back(
            {armor,
             "classifier(name/conf)",
             m,
             false,
             left_light.tilt_angle,
             right_light.tilt_angle});
          if (viz_show_number_img_ && viz_show_number_img_name_conf_reject_) {
            latest_number_img = armor.pattern.clone();
            latest_number_img_tag = "reject_name_conf";
          }
          tools::logger()->warn(
            "[PhoenixTraditionDetector][suspicious] reject=classifier(name/conf) "
            "name={} conf={:.2f} h_ratio={:.2f} angle_diff={:.2f} pair_angle={:.2f} center_dist={:.2f}",
            ARMOR_NAMES[static_cast<int>(armor.name)],
            armor.confidence,
            m.light_height_ratio,
            m.light_angle_diff,
            m.angle,
            m.light_center_distance);
        }
        continue;
      }

      armor.type =
        get_armor_type(armor, armor_ratio_big_threshold_, armor_ratio_small_threshold_);

      const bool type_ok = check_type(armor);
      if (!type_ok) {
        if (suspicious_debug_ && armor.confidence >= suspicious_min_confidence_) {
          suspicious_armors.push_back(
            {armor,
             "type_mismatch",
             m,
             false,
             left_light.tilt_angle,
             right_light.tilt_angle});
          if (viz_show_number_img_ && viz_show_number_img_type_reject_) {
            latest_number_img = armor.pattern.clone();
            latest_number_img_tag = "reject_type";
          }
          tools::logger()->warn(
            "[PhoenixTraditionDetector][suspicious] reject=type_mismatch "
            "name={} type={} conf={:.2f} ratio={:.2f} h_ratio={:.2f} center_dist={:.2f}",
            ARMOR_NAMES[static_cast<int>(armor.name)],
            ARMOR_TYPES[static_cast<int>(armor.type)],
            armor.confidence,
            armor.ratio,
            m.light_height_ratio,
            m.light_center_distance);
        }
        continue;
      }

      armor.center_norm = {
        armor.center.x / std::max(1, bgr_img.cols), armor.center.y / std::max(1, bgr_img.rows)};

      armors.push_back(std::move(armor));
    }
  }

  if (debug_img_) {
    if (viz_show_gray_binary_) {
      cv::imshow("phoenix_gray_binary", preprocessed_image_);
    }
    if (viz_show_light_binary_) {
      cv::imshow("phoenix_light_binary", light_contour_binary_image_);
    }
  }

  if (debug_) {
    cv::Mat vis = bgr_img.clone();
    if (viz_show_light_boxes_) {
      const cv::Scalar light_box_color = enemy_is_red_ ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
      for (const auto & L : lights) {
        std::vector<cv::Point> poly;
        poly.reserve(4);
        for (int i = 0; i < 4; ++i) {
          poly.emplace_back(cv::Point(static_cast<int>(L.point[i].x), static_cast<int>(L.point[i].y)));
        }
        cv::polylines(vis, poly, true, light_box_color, 2);
      }
    }
    if (viz_show_accepted_armors_) {
      for (const auto & a : armors) {
        tools::draw_points(vis, a.points, {0, 255, 0}, 2);
        if (a.points.size() == 4) {
          const auto label = cv::format("n=%d c=%.2f", static_cast<int>(a.name), a.confidence);
          const cv::Point anchor(
            static_cast<int>(a.points[3].x), static_cast<int>(a.points[3].y) + 12);
          tools::draw_text(vis, label, anchor, {0, 255, 0}, 0.45, 1);
        }
      }
    }
    if (viz_show_suspicious_armors_) {
      for (const auto & s : suspicious_armors) {
        // 分阶段着色：紫色=几何拒绝；黄色=分类阶段拒绝（name/conf/type）
        const bool geometry_reject =
          (s.reason.find("height_ratio") != std::string::npos) ||
          (s.reason.find("angle_diff") != std::string::npos) ||
          (s.reason.find("pair_angle") != std::string::npos) ||
          (s.reason.find("center_distance") != std::string::npos) ||
          (s.reason.find("contain_light") != std::string::npos);
        const bool name_conf_reject =
          s.reason.find("classifier(name/conf)") != std::string::npos;
        const bool type_reject = s.reason.find("type_mismatch") != std::string::npos;
        const bool classify_reject = !geometry_reject;

        bool draw_this = false;
        if (geometry_reject && viz_show_suspicious_geometry_) {
          draw_this = true;
        }
        if (classify_reject && viz_show_suspicious_classify_) {
          if (name_conf_reject) {
            draw_this = draw_this || viz_show_suspicious_name_conf_;
          } else if (type_reject) {
            draw_this = draw_this || viz_show_suspicious_type_;
          } else {
            draw_this = true;
          }
        }
        if (!draw_this) {
          continue;
        }

        const cv::Scalar suspicious_color = geometry_reject ? cv::Scalar(255, 0, 255)
                                                            : cv::Scalar(0, 255, 255);
        tools::draw_points(vis, s.armor.points, suspicious_color, 2);
        if (s.armor.points.size() == 4) {
          const auto label = cv::format(
            "n=%d c=%.2f", static_cast<int>(s.armor.name), s.armor.confidence);
          const cv::Point anchor(
            static_cast<int>(s.armor.points[3].x), static_cast<int>(s.armor.points[3].y) + 12);
          tools::draw_text(vis, label, anchor, suspicious_color, 0.45, 1);
        }
        if (viz_show_suspicious_reason_text_) {
          tools::draw_text(
            vis,
            "sus:" + std::string(ARMOR_NAMES[static_cast<int>(s.armor.name)]) + " " + s.reason,
            {static_cast<int>(s.armor.center.x), static_cast<int>(s.armor.center.y) - 10},
            suspicious_color,
            0.45,
            1);
        }
      }
    }
    cv::imshow("phoenix_tradition", vis);

    if (viz_show_number_img_ && !latest_number_img.empty()) {
      cv::Mat number_vis = latest_number_img.clone();
      cv::resize(number_vis, number_vis, {}, 4.0, 4.0, cv::INTER_NEAREST);
      tools::draw_text(
        number_vis, latest_number_img_tag, {6, 18}, {0, 255, 255}, 0.5, 1);
      cv::imshow("phoenix_number_img", number_vis);
    }
  }

  return armors;
}

}  // namespace auto_aim
