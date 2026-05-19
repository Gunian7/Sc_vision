#ifndef AUTO_AIM__YOLOV5_RKNN_HPP
#define AUTO_AIM__YOLOV5_RKNN_HPP

#include <deque>
#include <list>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/yolos/rknn_threadpool/include/postprocess.h"
#include "tasks/auto_aim/yolos/rknn_threadpool/include/rkYolov5s.hpp"
#include "tasks/auto_aim/yolos/rknn_threadpool/include/rknnPool.hpp"

namespace auto_aim {

class YOLOV5_RKNN: public YOLOBase {
public:
    using JobId                          = YOLO::JobId;
    static constexpr JobId kInvalidJobId = YOLO::kInvalidJobId;

    YOLOV5_RKNN(const std::string& config_path, bool debug);

    std::list<Armor> detect(const cv::Mat& bgr_img, int frame_count) override;

    std::list<Armor>
    postprocess(double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count) override;

    JobId submit(const cv::Mat& img, int frame_count = -1);
    std::list<Armor> wait(JobId job_id);
    bool try_wait(JobId job_id, std::list<Armor>& armors);

private:
    using Pool = rknnPool<rkYolov5s, cv::Mat, detect_result_group_t>;

    struct PendingJob {
        JobId job_id = kInvalidJobId;
        cv::Mat raw_img;
        int frame_count = -1;
    };

    std::string model_path_;
    std::string save_path_;
    bool debug_           = true;
    bool use_roi_         = false;
    bool use_traditional_ = false;

    const float nms_threshold_   = 0.3F;
    const float score_threshold_ = 0.7F;
    double min_confidence_       = 0.0;
    double binary_threshold_     = 0.0;

    cv::Rect roi_;
    cv::Point2f offset_;
    cv::Mat tmp_img_;

    Detector detector_;

    int thread_num_ = 3;
    std::unique_ptr<Pool> pool_;

    std::mutex state_mtx_;
    JobId next_job_id_ = 1;
    std::deque<PendingJob> pending_jobs_;
    std::unordered_map<JobId, std::list<Armor>> completed_jobs_;

    bool check_name(const Armor& armor) const;
    bool check_type(const Armor& armor) const;

    cv::Point2f get_center_norm(const cv::Mat& bgr_img, const cv::Point2f& center) const;
    cv::Rect get_active_roi(const cv::Mat& raw_img) const;

    std::list<Armor> parse(double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count);
    std::list<Armor>
    parse(const detect_result_group_t& output, const cv::Mat& bgr_img, int frame_count);

    bool collect_one(bool blocking);
    bool take_completed(JobId job_id, std::list<Armor>& armors);

    void save(const Armor& armor) const;
    void draw_detections(const cv::Mat& img, const std::list<Armor>& armors, int frame_count) const;
    double sigmoid(double x);
};

} // namespace auto_aim

#endif // AUTO_AIM__YOLOV5_RKNN_HPP
