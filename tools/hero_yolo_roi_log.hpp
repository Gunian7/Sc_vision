#ifndef TOOLS__HERO_YOLO_ROI_LOG_HPP
#define TOOLS__HERO_YOLO_ROI_LOG_HPP

#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace tools
{

/** 与 standard_mpc_se 一致：在全图上画出 YOLO 裁剪 ROI（青框 + 「ROI」标签，叠在重投影窗口上） */
inline void hero_draw_yolo_roi_overlay(cv::Mat & img, const auto_aim::YOLO & yolo)
{
  cv::Rect roi_rect;
  bool roi_active = false;
  if (!yolo.get_debug_roi(roi_rect, roi_active) || !roi_active || roi_rect.width <= 0 ||
      roi_rect.height <= 0) {
    return;
  }
  cv::rectangle(img, roi_rect, cv::Scalar(0, 255, 255), 2);
  tools::draw_text(
    img, "ROI", cv::Point(roi_rect.x, std::max(0, roi_rect.y - 8)), {0, 255, 255});
}

/** 与 hero_detectwithpnp_test 一致：每帧打 YOLO 配置的 ROI（get_debug_roi 为真时） */
inline void hero_log_yolo_roi(const auto_aim::YOLO & yolo)
{
  cv::Rect roi_rect;
  bool roi_active = false;
  if (yolo.get_debug_roi(roi_rect, roi_active)) {
    tools::logger()->info(
      "[ROI] active={}, x={}, y={}, w={}, h={}",
      roi_active,
      roi_rect.x,
      roi_rect.y,
      roi_rect.width,
      roi_rect.height);
  }
}

} // namespace tools

#endif // TOOLS__HERO_YOLO_ROI_LOG_HPP
