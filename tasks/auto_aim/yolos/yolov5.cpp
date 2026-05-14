#include "yolov5.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/resolve_path_relative_to_config.hpp"

namespace auto_aim
{
YOLOV5::YOLOV5(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  auto yolo_name = yaml["yolo_name"].as<std::string>();
  if (yolo_name == "rp24") {
    if (!yaml["rp24_model_path"].IsDefined()) {
      throw std::runtime_error("yolo_name=rp24 but rp24_model_path is not defined in yaml");
    }
    model_path_ = tools::resolve_path_relative_to_config(
        config_path, yaml["rp24_model_path"].as<std::string>());
  } else {
    model_path_ = tools::resolve_path_relative_to_config(
        config_path, yaml["yolov5_model_path"].as<std::string>());
  }
  device_ = yaml["device"].as<std::string>();
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();
  if (yaml["num_infer_requests"].IsDefined()) {
    num_requests_ = yaml["num_infer_requests"].as<int>();
  }
  if (yaml["use_dynamic_roi"].IsDefined()) {
    use_dynamic_roi_ = yaml["use_dynamic_roi"].as<bool>();
  }
  if (yaml["dynamic_roi"].IsDefined()) {
    use_dynamic_roi_ = yaml["dynamic_roi"].as<bool>();
  }
  if (yaml["dynamic_roi_shrink_frames"].IsDefined()) {
    dynamic_roi_shrink_frames_ = yaml["dynamic_roi_shrink_frames"].as<int>();
  }
  if (yaml["dynamic_roi_lost_frames"].IsDefined()) {
    dynamic_roi_lost_frames_ = yaml["dynamic_roi_lost_frames"].as<int>();
  }

  target_roi_ = cv::Rect(x, y, width, height);
  full_roi_ = cv::Rect(0, 0, 0, 0);

  // if (debug_) {
  //   tools::logger()->info("[YOLOV5] dynamic_roi={} shrink_frames={} lost_frames={} target_roi=[{}, {}, {}, {}]", 
  //     use_dynamic_roi_, dynamic_roi_shrink_frames_, dynamic_roi_lost_frames_,
  //     target_roi_.x, target_roi_.y, target_roi_.width, target_roi_.height);
  // }

  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);
  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  // TODO: ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));

  // create infer request pool
  infer_pool_ = std::make_unique<InferRequestPool>(compiled_model_, num_requests_);
}
  // // 多InferRequest
  // for (int i = 0; i < num_requests_; ++i) {
  //   infer_requests_.emplace_back(compiled_model_.create_infer_request());
  //   free_infer_request_indices_.push(i);
  // }
  // tools::logger()->info("[YOLOV5] initialized , num_requests: {}", num_requests_);

std::list<Armor> YOLOV5::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  if (use_dynamic_roi_ && !dynamic_roi_initialized_) {
    full_roi_ = cv::Rect(0, 0, raw_img.cols, raw_img.rows);
    // gate to ensure full image start state
    roi_ = full_roi_;
    use_roi_ = false;
    dynamic_roi_initialized_ = true;
  }

  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width == -1) {  // -1 表示该维度不裁切
      roi_.width = raw_img.cols;
    }
    if (roi_.height == -1) {  // -1 表示该维度不裁切
      roi_.height = raw_img.rows;
    }
    
    // Bounds check
    roi_.x = std::max(0, roi_.x);
    roi_.y = std::max(0, roi_.y);
    roi_.width = std::min(raw_img.cols - roi_.x, roi_.width);
    roi_.height = std::min(raw_img.rows - roi_.y, roi_.height);
    if (roi_.width <= 0 || roi_.height <= 0) {
      roi_ = cv::Rect(0, 0, raw_img.cols, raw_img.rows);
      use_roi_ = false;
      offset_ = cv::Point2f(0, 0);
      bgr_img = raw_img;
    } else {
      bgr_img = raw_img(roi_);
      offset_ = cv::Point2f(roi_.x, roi_.y);
    }
  } else {
    bgr_img = raw_img;
    offset_ = cv::Point2f(0, 0);
  }

  // if (debug_) {
  //   tools::logger()->info("[YOLOV5 ROI] use_roi={}, dynamic={}, rect=[{}, {}, {}, {}]", 
  //     use_roi_, use_dynamic_roi_, roi_.x, roi_.y, roi_.width, roi_.height);
  // }

  auto x_scale = static_cast<double>(640) / bgr_img.rows;
  auto y_scale = static_cast<double>(640) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  // preproces
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  // infer using pooled InferRequest
  if (!infer_pool_) {
    tools::logger()->warn("Infer pool not initialized, dropping frame {}", frame_count);
    return std::list<Armor>();
  }

  auto lease = infer_pool_->acquire_lease();
  if (!lease.valid()) {
    tools::logger()->warn("No free infer request available, dropping frame {}", frame_count);
    return std::list<Armor>();
  }

  auto & infer_request = lease.request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  auto armors = parse(scale, output, raw_img, frame_count);

  if (use_dynamic_roi_) {
    if (armors.empty()) {
      consecutive_lost_frames_++;
      consecutive_tracking_frames_ = 0;
      if (consecutive_lost_frames_ > dynamic_roi_lost_frames_) {
        roi_ = full_roi_;
        use_roi_ = false;
      }
    } else {
      consecutive_tracking_frames_++;
      consecutive_lost_frames_ = 0;
      float progress = std::min(1.0f, static_cast<float>(consecutive_tracking_frames_) /
        dynamic_roi_shrink_frames_);
      int new_x = static_cast<int>(full_roi_.x + (target_roi_.x - full_roi_.x) * progress);
      int new_y = static_cast<int>(full_roi_.y + (target_roi_.y - full_roi_.y) * progress);
      int new_w = static_cast<int>(full_roi_.width + (target_roi_.width - full_roi_.width) * progress);
      int new_h = static_cast<int>(full_roi_.height + (target_roi_.height - full_roi_.height) * progress);

      new_x = std::clamp(new_x, 0, raw_img.cols - new_w);
      new_y = std::clamp(new_y, 0, raw_img.rows - new_h);
      new_w = std::clamp(new_w, 1, raw_img.cols - new_x);
      new_h = std::clamp(new_h, 1, raw_img.rows - new_y);

      roi_ = cv::Rect(new_x, new_y, new_w, new_h);
      use_roi_ = true;
    }
  }

  return armors;
}

std::list<Armor> YOLOV5::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // for each row: xywh + classess
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);

    if (score < score_threshold_) continue;

    std::vector<cv::Point2f> armor_key_points;

    //颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);     //color
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  //num
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    _class_id = class_id.x;
    _color_id = color_id.x;

    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

    for (int i = 1; i < armor_key_points.size(); i++) {
      if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    if (use_roi_) {
      armors.emplace_back(
        color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    // 使用传统方法二次矫正角点
    if (use_traditional_) detector_.detect(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

bool YOLOV5::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于神经网络的迭代
  // if (name_ok && !confidence_ok) save(armor);

  return name_ok && confidence_ok;
}

bool YOLOV5::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  // 保存异常的图案，用于神经网络的迭代
  // if (!name_ok) save(armor);

  return name_ok;
}

cv::Point2f YOLOV5::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLOV5::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.box.tl() + cv::Point(0, -15), {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 0.8, 0.8);  // 显示时缩小图片尺寸
  cv::imshow("detection", detection);
}

void YOLOV5::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, ARMOR_NAMES[armor.name], file_name);
  cv::imwrite(img_path, tmp_img_);
}

double YOLOV5::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

std::list<Armor> YOLOV5::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

bool YOLOV5::get_debug_roi(cv::Rect & roi, bool & active) const
{
  roi = roi_;
  active = use_roi_;
  return use_roi_;
}

}  // namespace auto_aim