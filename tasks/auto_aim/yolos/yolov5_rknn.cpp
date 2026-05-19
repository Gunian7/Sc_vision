#include "yolov5_rknn.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim {

YOLOV5_RKNN::YOLOV5_RKNN(const std::string& config_path, bool debug):
    debug_(debug),
    detector_(config_path, false) {
    auto yaml = YAML::LoadFile(config_path);

    model_path_       = yaml["yolov5_rknn_model_path"].as<std::string>();
    binary_threshold_ = yaml["threshold"].as<double>();
    min_confidence_   = yaml["min_confidence"].as<double>();
    use_roi_          = yaml["use_roi"].as<bool>();
    use_traditional_  = yaml["use_traditional"].as<bool>();
    thread_num_ = yaml["yolov5_rknn_thread_num"] ? yaml["yolov5_rknn_thread_num"].as<int>() : 3;

    int x      = yaml["roi"]["x"].as<int>();
    int y      = yaml["roi"]["y"].as<int>();
    int width  = yaml["roi"]["width"].as<int>();
    int height = yaml["roi"]["height"].as<int>();
    roi_       = cv::Rect(x, y, width, height);
    offset_    = cv::Point2f(x, y);

    save_path_ = "imgs";
    std::filesystem::create_directory(save_path_);

    pool_ = std::make_unique<Pool>(model_path_, thread_num_);
    if (pool_->init() != 0) {
        throw std::runtime_error("Failed to initialize RKNN thread pool");
    }

    tools::logger()->info("[YOLOV5_RKNN] initialized, thread_num={}", thread_num_);
}

std::list<Armor> YOLOV5_RKNN::detect(const cv::Mat& raw_img, int frame_count) {
    auto job_id = submit(raw_img, frame_count);
    if (job_id == kInvalidJobId) {
        return {};
    }
    return wait(job_id);
}

std::list<Armor>
YOLOV5_RKNN::postprocess(double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count) {
    return parse(scale, output, bgr_img, frame_count);
}

YOLOV5_RKNN::JobId YOLOV5_RKNN::submit(const cv::Mat& raw_img, int frame_count) {
    if (raw_img.empty()) {
        tools::logger()->warn("[YOLOV5_RKNN] Empty img!, camera drop!");
        return kInvalidJobId;
    }

    cv::Mat infer_img;
    if (use_roi_) {
        auto roi = get_active_roi(raw_img);
        if (roi.width <= 0 || roi.height <= 0) {
            tools::logger()->warn("[YOLOV5_RKNN] Invalid ROI, fallback to full image");
            infer_img = raw_img.clone();
        } else {
            infer_img = raw_img(roi).clone();
        }
    } else {
        infer_img = raw_img.clone();
    }

    if (pool_->put(infer_img) != 0) {
        tools::logger()->error("[YOLOV5_RKNN] Failed to submit inference task");
        return kInvalidJobId;
    }

    std::lock_guard<std::mutex> lock(state_mtx_);
    auto job_id = next_job_id_++;
    pending_jobs_.push_back({ job_id, raw_img.clone(), frame_count });
    return job_id;
}

std::list<Armor> YOLOV5_RKNN::wait(JobId job_id) {
    if (job_id == kInvalidJobId) {
        return {};
    }

    std::list<Armor> armors;
    if (take_completed(job_id, armors)) {
        return armors;
    }

    while (collect_one(true)) {
        if (take_completed(job_id, armors)) {
            return armors;
        }
    }

    tools::logger()->warn("[YOLOV5_RKNN] wait() failed for job_id={}", job_id);
    return {};
}

bool YOLOV5_RKNN::try_wait(JobId job_id, std::list<Armor>& armors) {
    if (job_id == kInvalidJobId) {
        return false;
    }

    if (take_completed(job_id, armors)) {
        return true;
    }

    while (collect_one(false)) {
        if (take_completed(job_id, armors)) {
            return true;
        }
    }

    return false;
}

bool YOLOV5_RKNN::check_name(const Armor& armor) const {
    auto name_ok       = armor.name != ArmorName::not_armor;
    auto confidence_ok = armor.confidence > min_confidence_;
    return name_ok && confidence_ok;
}

bool YOLOV5_RKNN::check_type(const Armor& armor) const {
    auto name_ok = (armor.type == ArmorType::small)
        ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
        : (armor.name != ArmorName::two && armor.name != ArmorName::sentry
           && armor.name != ArmorName::outpost);
    return name_ok;
}

cv::Point2f YOLOV5_RKNN::get_center_norm(const cv::Mat& bgr_img, const cv::Point2f& center) const {
    auto h = bgr_img.rows;
    auto w = bgr_img.cols;
    return { center.x / w, center.y / h };
}

cv::Rect YOLOV5_RKNN::get_active_roi(const cv::Mat& raw_img) const {
    cv::Rect roi = roi_;
    if (roi.width == -1) {
        roi.width = raw_img.cols;
    }
    if (roi.height == -1) {
        roi.height = raw_img.rows;
    }

    roi &= cv::Rect(0, 0, raw_img.cols, raw_img.rows);
    return roi;
}

std::list<Armor>
YOLOV5_RKNN::parse(double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count) {
    std::vector<int> color_ids, num_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> armors_key_points;
    for (int r = 0; r < output.rows; r++) {
        double score = output.at<float>(r, 8);
        score        = sigmoid(score);

        if (score < score_threshold_)
            continue;

        std::vector<cv::Point2f> armor_key_points;

        cv::Mat color_scores   = output.row(r).colRange(9, 13);
        cv::Mat classes_scores = output.row(r).colRange(13, 22);
        cv::Point class_id, color_id;
        int _class_id, _color_id;
        double score_color, score_num;
        cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
        cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
        _class_id = class_id.x;
        _color_id = color_id.x;

        armor_key_points.push_back(
            cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale)
        );
        armor_key_points.push_back(
            cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale)
        );
        armor_key_points.push_back(
            cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale)
        );
        armor_key_points.push_back(
            cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale)
        );

        float min_x = armor_key_points[0].x;
        float max_x = armor_key_points[0].x;
        float min_y = armor_key_points[0].y;
        float max_y = armor_key_points[0].y;

        for (size_t i = 1; i < armor_key_points.size(); i++) {
            if (armor_key_points[i].x < min_x)
                min_x = armor_key_points[i].x;
            if (armor_key_points[i].x > max_x)
                max_x = armor_key_points[i].x;
            if (armor_key_points[i].y < min_y)
                min_y = armor_key_points[i].y;
            if (armor_key_points[i].y > max_y)
                max_y = armor_key_points[i].y;
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
    for (const auto& i: indices) {
        if (use_roi_) {
            armors.emplace_back(
                color_ids[i],
                num_ids[i],
                confidences[i],
                boxes[i],
                armors_key_points[i],
                offset_
            );
        } else {
            armors.emplace_back(
                color_ids[i],
                num_ids[i],
                confidences[i],
                boxes[i],
                armors_key_points[i]
            );
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

        if (use_traditional_)
            detector_.detect(*it, bgr_img);
        it->center_norm = get_center_norm(bgr_img, it->center);
        ++it;
    }

    if (debug_)
        draw_detections(bgr_img, armors, frame_count);

    return armors;
}

std::list<Armor>
YOLOV5_RKNN::parse(const detect_result_group_t& output, const cv::Mat& bgr_img, int frame_count) {
    std::list<Armor> armors;
    for (int i = 0; i < output.count; i++) {
        const auto& det = output.results[i];
        std::vector<cv::Point2f> armor_keypoints;
        armor_keypoints.reserve(NUM_KEYPOINTS);
        for (int kp = 0; kp < NUM_KEYPOINTS; kp++) {
            armor_keypoints.emplace_back(det.keypoints[kp].x, det.keypoints[kp].y);
        }

        const int x1 = static_cast<int>(std::floor(det.bbox_xyxy[0]));
        const int y1 = static_cast<int>(std::floor(det.bbox_xyxy[1]));
        const int x2 = static_cast<int>(std::ceil(det.bbox_xyxy[2]));
        const int y2 = static_cast<int>(std::ceil(det.bbox_xyxy[3]));
        cv::Rect box(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));

        if (use_roi_) {
            armors.emplace_back(det.color_id, det.label, det.prob, box, armor_keypoints, offset_);
        } else {
            armors.emplace_back(det.color_id, det.label, det.prob, box, armor_keypoints);
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

        if (use_traditional_)
            detector_.detect(*it, bgr_img);
        it->center_norm = get_center_norm(bgr_img, it->center);
        ++it;
    }

    if (debug_)
        draw_detections(bgr_img, armors, frame_count);

    return armors;
}

bool YOLOV5_RKNN::collect_one(bool blocking) {
    PendingJob meta;
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        if (pending_jobs_.empty()) {
            return false;
        }
    }

    detect_result_group_t output {};
    int ret = blocking ? pool_->get(output) : pool_->try_get(output);
    if (ret != 0) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        if (pending_jobs_.empty()) {
            return false;
        }
        meta = std::move(pending_jobs_.front());
        pending_jobs_.pop_front();
    }

    auto armors = parse(output, meta.raw_img, meta.frame_count);

    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        completed_jobs_[meta.job_id] = std::move(armors);
    }

    return true;
}

bool YOLOV5_RKNN::take_completed(JobId job_id, std::list<Armor>& armors) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    auto it = completed_jobs_.find(job_id);
    if (it == completed_jobs_.end()) {
        return false;
    }

    armors = std::move(it->second);
    completed_jobs_.erase(it);
    return true;
}

void YOLOV5_RKNN::save(const Armor& armor) const {
    auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    auto img_path  = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
    cv::imwrite(img_path, tmp_img_);
}

void YOLOV5_RKNN::draw_detections(
    const cv::Mat& img,
    const std::list<Armor>& armors,
    int frame_count
) const {
    auto detection = img.clone();
    tools::draw_text(detection, fmt::format("[{}]", frame_count), { 10, 30 }, { 255, 255, 255 });
    for (const auto& armor: armors) {
        auto info = fmt::format(
            "{:.2f} {} {} {}",
            armor.confidence,
            COLORS[armor.color],
            ARMOR_NAMES[armor.name],
            ARMOR_TYPES[armor.type]
        );
        tools::draw_points(detection, armor.points, { 0, 255, 0 });
        tools::draw_text(detection, info, armor.center, { 0, 255, 0 });
    }

    if (use_roi_) {
        cv::rectangle(detection, get_active_roi(img), cv::Scalar(0, 255, 0), 2);
    }
    cv::resize(detection, detection, {}, 0.5, 0.5);
}

double YOLOV5_RKNN::sigmoid(double x) {
    if (x > 0) {
        return 1.0 / (1.0 + std::exp(-x));
    }
    return std::exp(x) / (1.0 + std::exp(x));
}

} // namespace auto_aim
