#ifndef AUTO_AIM__PHOENIX_TRADITION_DETECTOR_HPP
#define AUTO_AIM__PHOENIX_TRADITION_DETECTOR_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.hpp"
#include "classifier.hpp"

namespace auto_aim
{

/** armor_detector 风格灯条检测（通道差分 + 亮度二值交集）+ Sc_vision Classifier 数字分类。
 *  供任意链接 auto_aim 的目标使用；是否启用由 yaml 根键 use_phoenix_traditional 等在上层决定。 */
class PhoenixTraditionDetector
{
public:
  explicit PhoenixTraditionDetector(const std::string & config_path, bool debug = false);

  std::list<Armor> detect(
    const cv::Mat & bgr_img, int frame_count = -1,
    std::vector<Armor> * name_conf_rejected_armors = nullptr);

private:
  Classifier classifier_;

  int binary_threshold_[2];
  int light_contour_threshold_[2];
  int dilate_kernel_size_;
  int min_contour_points_{10};
  double light_min_ratio_{1.5};
  double light_max_tilt_angle_deg_{25.0};
  double pair_light_height_ratio_min_{0.8};
  double pair_light_height_ratio_max_{1.2};
  double pair_light_angle_diff_max_deg_{10.0};
  double pair_center_line_angle_max_deg_{30.0};
  double pair_center_distance_small_min_{0.8};
  double pair_center_distance_small_max_{3.2};
  double pair_center_distance_big_min_{3.2};
  double pair_center_distance_big_max_{5.5};
  bool enable_contain_light_filter_{true};
  bool viz_show_gray_binary_{true};
  bool viz_show_light_binary_{true};
  bool viz_show_light_boxes_{true};
  bool viz_show_accepted_armors_{true};
  bool viz_show_suspicious_armors_{true};
  bool viz_show_suspicious_geometry_{true};
  bool viz_show_suspicious_classify_{true};
  bool viz_show_suspicious_name_conf_{true};
  bool viz_show_suspicious_type_{true};
  bool viz_show_suspicious_reason_text_{true};
  bool viz_show_number_img_{true};
  bool viz_show_number_img_geometry_reject_{false};
  bool viz_show_number_img_name_conf_reject_{true};
  bool viz_show_number_img_type_reject_{true};
  bool suspicious_debug_{false};
  double suspicious_min_confidence_{0.0};
  double armor_ratio_big_threshold_{3.0};
  double armor_ratio_small_threshold_{2.5};
  bool enemy_is_red_{false};
  double min_confidence_{0.7};
  bool debug_;
  /** 来自 yaml debug_img：显示灯条二值化等调试图（与 debug_ 独立，可仅看二值） */
  bool debug_img_{false};

  cv::Mat kernel_;
  cv::Mat preprocessed_image_;
  cv::Mat channels_[3];
  cv::Mat color_mask_;
  cv::Mat light_contour_binary_image_;

  cv::Mat preprocess_image(const cv::Mat & input) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PHOENIX_TRADITION_DETECTOR_HPP
