#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <cstdint>
#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace auto_aim
{
class YOLOBase
{
public:
  virtual ~YOLOBase() = default;

  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;

  virtual bool get_debug_roi(cv::Rect & roi, bool & active) const
  {
    (void)roi;
    (void)active;
    return false;
  }
};

class YOLO
{
public:
  using JobId = uint64_t;
  static constexpr JobId kInvalidJobId = 0;

  YOLO(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

  JobId submit(const cv::Mat & img, int frame_count = -1);
  std::list<Armor> wait(JobId job_id);
  bool try_wait(JobId job_id, std::list<Armor> & armors);

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

  bool get_debug_roi(cv::Rect & roi, bool & active) const;

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
