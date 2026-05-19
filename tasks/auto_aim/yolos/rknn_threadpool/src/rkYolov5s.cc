#include "rknn_api.h"
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <stdio.h>

#include "postprocess.h"
#include "preprocess.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "coreNum.hpp"
#include "rkYolov5s.hpp"

namespace {
std::atomic<uint64_t> g_infer_seq { 0 };
static const bool g_enable_timing = []() {
    const char* env = std::getenv("RKNN_TIMING");
    return env != nullptr && env[0] != '\0' && env[0] != '0';
}();

static inline double elapsed_ms(
    const std::chrono::steady_clock::time_point& start,
    const std::chrono::steady_clock::time_point& end
) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}
} // namespace

static void dump_tensor_attr(rknn_tensor_attr* attr) {
    std::string shape_str = attr->n_dims < 1 ? "" : std::to_string(attr->dims[0]);
    for (int i = 1; i < attr->n_dims; ++i) {
        shape_str += ", " + std::to_string(attr->dims[i]);
    }

    // printf("  index=%d, name=%s, n_dims=%d, dims=[%s], n_elems=%d, size=%d, w_stride = %d, size_with_stride=%d, fmt=%s, "
    //        "type=%s, qnt_type=%s, "
    //        "zp=%d, scale=%f\n",
    //        attr->index, attr->name, attr->n_dims, shape_str.c_str(), attr->n_elems, attr->size, attr->w_stride,
    //        attr->size_with_stride, get_format_string(attr->fmt), get_type_string(attr->type),
    //        get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

static unsigned char* load_data(FILE* fp, size_t ofst, size_t sz) {
    unsigned char* data;
    int ret;

    data = NULL;

    if (NULL == fp) {
        return NULL;
    }

    ret = fseek(fp, ofst, SEEK_SET);
    if (ret != 0) {
        printf("blob seek failure.\n");
        return NULL;
    }

    data = (unsigned char*)malloc(sz);
    if (data == NULL) {
        printf("buffer malloc failure.\n");
        return NULL;
    }
    ret = fread(data, 1, sz, fp);
    return data;
}

static unsigned char* load_model(const char* filename, int* model_size) {
    FILE* fp;
    unsigned char* data;

    fp = fopen(filename, "rb");
    if (NULL == fp) {
        printf("Open file %s failed.\n", filename);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);

    data = load_data(fp, 0, size);

    fclose(fp);

    *model_size = size;
    return data;
}

static int saveFloat(const char* file_name, float* output, int element_size) {
    FILE* fp;
    fp = fopen(file_name, "w");
    for (int i = 0; i < element_size; i++) {
        fprintf(fp, "%.6f\n", output[i]);
    }
    fclose(fp);
    return 0;
}

rkYolov5s::rkYolov5s(const std::string& model_path) {
    this->model_path   = model_path;
    nms_threshold      = NMS_THRESH;  // 默认的NMS阈值
    box_conf_threshold = CONF_THRESH; // 默认的置信度阈值
    detect_color       = -1;          // -1=all, 0=blue, 1=red
}

int rkYolov5s::init(rknn_context* ctx_in, bool share_weight) {
    printf("Loading model...\n");
    int model_data_size = 0;
    model_data          = load_model(model_path.c_str(), &model_data_size);
    // 模型参数复用/Model parameter reuse
    if (share_weight == true)
        ret = rknn_dup_context(ctx_in, &ctx);
    else
        ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
    if (ret < 0) {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }

    // 设置模型绑定的核心/Set the core of the model that needs to be bound
    rknn_core_mask core_mask;
    switch (get_core_num()) {
        case 0:
            core_mask = RKNN_NPU_CORE_0;
            break;
        case 1:
            core_mask = RKNN_NPU_CORE_1;
            break;
        case 2:
            core_mask = RKNN_NPU_CORE_2;
            break;
    }
    ret = rknn_set_core_mask(ctx, core_mask);
    if (ret < 0) {
        printf("rknn_init core error ret=%d\n", ret);
        return -1;
    }

    rknn_sdk_version version;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret < 0) {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }
    printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);

    // 获取模型输入输出参数/Obtain the input and output parameters of the model
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0) {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    // 设置输入参数/Set the input parameters
    input_attrs = (rknn_tensor_attr*)calloc(io_num.n_input, sizeof(rknn_tensor_attr));
    for (int i = 0; i < io_num.n_input; i++) {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret < 0) {
            printf("rknn_init error ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(input_attrs[i]));
    }

    // 设置输出参数/Set the output parameters
    output_attrs = (rknn_tensor_attr*)calloc(io_num.n_output, sizeof(rknn_tensor_attr));
    for (int i = 0; i < io_num.n_output; i++) {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        dump_tensor_attr(&(output_attrs[i]));
    }

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
        printf("model is NCHW input fmt\n");
        channel = input_attrs[0].dims[1];
        height  = input_attrs[0].dims[2];
        width   = input_attrs[0].dims[3];
    } else {
        printf("model is NHWC input fmt\n");
        height  = input_attrs[0].dims[1];
        width   = input_attrs[0].dims[2];
        channel = input_attrs[0].dims[3];
    }
    printf("model input height=%d, width=%d, channel=%d\n", height, width, channel);

    memset(inputs, 0, sizeof(inputs));
    inputs[0].index        = 0;
    inputs[0].type         = RKNN_TENSOR_UINT8;
    inputs[0].size         = width * height * channel;
    inputs[0].fmt          = RKNN_TENSOR_NHWC;
    inputs[0].pass_through = 0;

    return 0;
}

rknn_context* rkYolov5s::get_pctx() {
    return &ctx;
}

detect_result_group_t rkYolov5s::infer(cv::Mat& orig_img) {
    std::lock_guard<std::mutex> lock(mtx);
    const uint64_t infer_seq = g_infer_seq.fetch_add(1, std::memory_order_relaxed) + 1;
    const auto t0            = std::chrono::steady_clock::now();
    cv::Mat img;
    cv::cvtColor(orig_img, img, cv::COLOR_BGR2RGB);
    const auto t1 = std::chrono::steady_clock::now();
    img_width     = img.cols;
    img_height    = img.rows;

    BOX_RECT pads;
    memset(&pads, 0, sizeof(BOX_RECT));
    cv::Size target_size(width, height);
    cv::Mat resized_img(target_size.height, target_size.width, CV_8UC3);
    // 计算缩放比例/Calculate the scaling ratio
    float scale_w = (float)target_size.width / img.cols;
    float scale_h = (float)target_size.height / img.rows;

    // 图像缩放/Image scaling
    if (img_width != width || img_height != height) {
        // rga
        rga_buffer_t src;
        rga_buffer_t dst;
        memset(&src, 0, sizeof(src));
        memset(&dst, 0, sizeof(dst));
        ret = resize_rga(src, dst, img, resized_img, target_size);
        if (ret != 0) {
            fprintf(stderr, "resize with rga error\n");
        }
        
        // // opencv
        // float min_scale = std::min(scale_w, scale_h);
        // scale_w = min_scale;
        // scale_h = min_scale;
        // letterbox(img, resized_img, pads, min_scale, target_size);
        
        inputs[0].buf = resized_img.data;
    } else {
        inputs[0].buf = img.data;
    }
    const auto t2 = std::chrono::steady_clock::now();

    rknn_inputs_set(ctx, io_num.n_input, inputs);
    const auto t3 = std::chrono::steady_clock::now();

    rknn_output outputs[io_num.n_output];
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < io_num.n_output; i++) {
        outputs[i].want_float = 1;
    }

    // 模型推理/Model inference
    ret           = rknn_run(ctx, NULL);
    const auto t4 = std::chrono::steady_clock::now();
    ret           = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);
    const auto t5 = std::chrono::steady_clock::now();

    // 后处理/Post-processing
    detect_result_group_t detect_result_group;
    memset(&detect_result_group, 0, sizeof(detect_result_group));
    detect_result_group.id = static_cast<int>(infer_seq);
    const auto t5_1        = std::chrono::steady_clock::now();
    post_process(
        (float*)outputs[0].buf,
        (float*)outputs[1].buf,
        (float*)outputs[2].buf,
        output_attrs,
        io_num.n_output,
        height,
        width,
        box_conf_threshold,
        nms_threshold,
        pads,
        scale_w,
        scale_h,
        &detect_result_group,
        detect_color
    );
    const auto t6 = std::chrono::steady_clock::now();

    // 绘制框体/Draw the box
    // char text[256];
    // for (int i = 0; i < detect_result_group.count; i++)
    // {
    //     detect_object_t *det = &(detect_result_group.results[i]);

    //     // Bbox
    //     int x1 = (int)det->bbox_xyxy[0];
    //     int y1 = (int)det->bbox_xyxy[1];
    //     int x2 = (int)det->bbox_xyxy[2];
    //     int y2 = (int)det->bbox_xyxy[3];
    //     cv::rectangle(orig_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

    //     // Keypoints
    //     for (int kp = 0; kp < NUM_KEYPOINTS; kp++)
    //     {
    //         cv::circle(orig_img, cv::Point((int)det->keypoints[kp].x, (int)det->keypoints[kp].y),
    //                    2, cv::Scalar(0, 0, 255), -1);
    //     }

    //     // Label text: cid=X cls=Y p=Z
    //     sprintf(text, "cid=%d cls=%d p=%.2f", det->color_id, det->label, det->prob);
    //     cv::putText(orig_img, text, cv::Point(x1, std::max(0, y1 - 6)),
    //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    // }

    ret           = rknn_outputs_release(ctx, io_num.n_output, outputs);
    const auto t7 = std::chrono::steady_clock::now();

    if (g_enable_timing) {
        printf(
            "[infer_timing] seq=%llu cvt=%.3fms resize=%.3fms input_set=%.3fms run=%.3fms outputs_get=%.3fms post_prep=%.3fms post_core=%.3fms post_total=%.3fms release=%.3fms total=%.3fms det=%d\n",
            (unsigned long long)infer_seq,
            elapsed_ms(t0, t1),
            elapsed_ms(t1, t2),
            elapsed_ms(t2, t3),
            elapsed_ms(t3, t4),
            elapsed_ms(t4, t5),
            elapsed_ms(t5, t5_1),
            elapsed_ms(t5_1, t6),
            elapsed_ms(t5, t6),
            elapsed_ms(t6, t7),
            elapsed_ms(t0, t7),
            detect_result_group.count
        );
    }

    return detect_result_group;
}

rkYolov5s::~rkYolov5s() {
    deinitPostProcess();

    ret = rknn_destroy(ctx);

    if (model_data)
        free(model_data);

    if (input_attrs)
        free(input_attrs);
    if (output_attrs)
        free(output_attrs);
}
