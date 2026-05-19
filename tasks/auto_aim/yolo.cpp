#include "yolo.hpp"

#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include "yolos/yolo11.hpp"
#include "yolos/yolov5.hpp"
#include "yolos/yolov5_rknn.hpp"
#include "yolos/yolov8.hpp"

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  if (yolo_name == "yolov8") {
    yolo_ = std::make_unique<YOLOV8>(config_path, debug);
  }
  else if (yolo_name == "yolo11") {
    yolo_ = std::make_unique<YOLO11>(config_path, debug);
  }
  else if (yolo_name == "yolov5") {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);
  }
  else if (yolo_name == "rp24") {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);
  }
  else if (yolo_name == "yolov5_rknn") {
    yolo_ = std::make_unique<YOLOV5_RKNN>(config_path, debug);
  }
  else {
    throw std::runtime_error("Unknown yolo name: " + yolo_name + "!");
  }
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

YOLO::JobId YOLO::submit(const cv::Mat & img, int frame_count)
{
  auto * rknn = dynamic_cast<YOLOV5_RKNN *>(yolo_.get());
  if (!rknn) {
    throw std::runtime_error("YOLO::submit is only supported by yolov5_rknn");
  }
  return rknn->submit(img, frame_count);
}

std::list<Armor> YOLO::wait(JobId job_id)
{
  auto * rknn = dynamic_cast<YOLOV5_RKNN *>(yolo_.get());
  if (!rknn) {
    throw std::runtime_error("YOLO::wait is only supported by yolov5_rknn");
  }
  return rknn->wait(job_id);
}

bool YOLO::try_wait(JobId job_id, std::list<Armor> & armors)
{
  auto * rknn = dynamic_cast<YOLOV5_RKNN *>(yolo_.get());
  if (!rknn) {
    throw std::runtime_error("YOLO::try_wait is only supported by yolov5_rknn");
  }
  return rknn->try_wait(job_id, armors);
}

bool YOLO::get_debug_roi(cv::Rect & roi, bool & active) const
{
  return yolo_->get_debug_roi(roi, active);
}

}  // namespace auto_aim
