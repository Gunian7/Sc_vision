// Copyright (c) 2021 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "postprocess.h"

#include <chrono>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <array>
#include <vector>

// Anchor definitions (same as inference_example.py)
static const float anchors[9][2] = {
    {10, 13}, {16, 30}, {33, 23},   // scale 0 (stride 8)
    {30, 61}, {62, 45}, {59, 119},  // scale 1 (stride 16)
    {116, 90}, {156, 198}, {373, 326} // scale 2 (stride 32)
};
static const int anchor_masks[3][3] = {
    {0, 1, 2}, {3, 4, 5}, {6, 7, 8}
};
static const int strides[3] = {8, 16, 32};

// -------------------------------------------------------
// Utility functions
// -------------------------------------------------------

static inline float sigmoid(float x)
{
    if (x > 50.0f) return 1.0f;
    if (x < -50.0f) return 0.0f;
    return 1.0f / (1.0f + expf(-x));
}

static inline float logit_clamp(float p)
{
    const float eps = 1e-6f;
    if (p < eps) p = eps;
    if (p > 1.0f - eps) p = 1.0f - eps;
    return logf(p / (1.0f - p));
}

// IoU between two xyxy boxes
static float iou_xyxy(const float *a, const float *b)
{
    float xx1 = fmaxf(a[0], b[0]);
    float yy1 = fmaxf(a[1], b[1]);
    float xx2 = fminf(a[2], b[2]);
    float yy2 = fminf(a[3], b[3]);
    float w = fmaxf(0.0f, xx2 - xx1);
    float h = fmaxf(0.0f, yy2 - yy1);
    float inter = w * h;
    float area1 = fmaxf(0.0f, a[2] - a[0]) * fmaxf(0.0f, a[3] - a[1]);
    float area2 = fmaxf(0.0f, b[2] - b[0]) * fmaxf(0.0f, b[3] - b[1]);
    float uni = area1 + area2 - inter + 1e-9f;
    return inter / uni;
}

struct Candidate
{
    float kpts[NUM_KEYPOINTS * 2];
    float bbox[4];
    float conf;
    float score_num;
    float score_color;
    int color_id;
    int num_id;
};

static inline void get_output_hwc(const rknn_tensor_attr &attr,
                                  int fallback_h,
                                  int fallback_w,
                                  int &h,
                                  int &w,
                                  int &c)
{
    h = fallback_h;
    w = fallback_w;
    c = NUM_ANCHORS_PER_SCALE * NUM_CHANNELS_PER_ANCHOR;

    if (attr.n_dims < 4)
    {
        return;
    }

    if (attr.fmt == RKNN_TENSOR_NHWC)
    {
        h = attr.dims[1];
        w = attr.dims[2];
        c = attr.dims[3];
    }
    else
    {
        c = attr.dims[1];
        h = attr.dims[2];
        w = attr.dims[3];
    }
}

static inline float read_output_value(const float *data,
                                      const rknn_tensor_attr &attr,
                                      int anchor_idx,
                                      int channel_idx,
                                      int y,
                                      int x,
                                      int h,
                                      int w)
{
    const int channels_per_scale = NUM_ANCHORS_PER_SCALE * NUM_CHANNELS_PER_ANCHOR;
    if (attr.fmt == RKNN_TENSOR_NHWC)
    {
        const size_t spatial_idx = (size_t)y * (size_t)w + (size_t)x;
        const size_t channel = (size_t)anchor_idx * NUM_CHANNELS_PER_ANCHOR + (size_t)channel_idx;
        return data[spatial_idx * (size_t)channels_per_scale + channel];
    }

    const size_t hw = (size_t)h * (size_t)w;
    const size_t anchor_base = (size_t)anchor_idx * NUM_CHANNELS_PER_ANCHOR * hw;
    const size_t spatial_idx = (size_t)y * (size_t)w + (size_t)x;
    return data[anchor_base + (size_t)channel_idx * hw + spatial_idx];
}

static void nms_candidates(const std::vector<Candidate> &candidates,
                           float iou_thresh,
                           std::vector<int> &keep)
{
    const int n = (int)candidates.size();
    keep.clear();
    if (n <= 0)
    {
        return;
    }

    thread_local std::vector<int> order;
    thread_local std::vector<uint8_t> suppressed;

    order.resize(n);
    suppressed.assign(n, 0);
    for (int i = 0; i < n; ++i)
    {
        order[i] = i;
    }

    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return candidates[a].score_num > candidates[b].score_num;
    });

    keep.reserve(n);
    for (int ii = 0; ii < n; ++ii)
    {
        const int i = order[ii];
        if (suppressed[i]) continue;
        keep.push_back(i);

        for (int jj = ii + 1; jj < n; ++jj)
        {
            const int j = order[jj];
            if (suppressed[j]) continue;
            const float iou = iou_xyxy(candidates[i].bbox, candidates[j].bbox);
            if (iou > iou_thresh)
            {
                suppressed[j] = 1;
            }
        }
    }
}

// -------------------------------------------------------
// Main post_process: 3-head float32 -> detect_result_group_t
// Matches Python postprocess_single_head exactly
// -------------------------------------------------------
int post_process(float *input0, float *input1, float *input2,
                 const rknn_tensor_attr *output_attrs, int output_count,
                 int model_in_h, int model_in_w,
                 float conf_threshold, float nms_threshold,
                 BOX_RECT pads, float scale_w, float scale_h,
                 detect_result_group_t *group,
                 int detect_color)
{
    const auto t0 = std::chrono::steady_clock::now();
    memset(group, 0, sizeof(detect_result_group_t));

    const int max_candidates =
        (model_in_h / strides[0]) * (model_in_w / strides[0]) * NUM_ANCHORS_PER_SCALE +
        (model_in_h / strides[1]) * (model_in_w / strides[1]) * NUM_ANCHORS_PER_SCALE +
        (model_in_h / strides[2]) * (model_in_w / strides[2]) * NUM_ANCHORS_PER_SCALE;

    thread_local std::vector<Candidate> candidates;
    thread_local std::vector<int> keep;

    candidates.clear();
    keep.clear();
    if ((int)candidates.capacity() < max_candidates)
    {
        candidates.reserve(max_candidates);
    }
    if ((int)keep.capacity() < max_candidates)
    {
        keep.reserve(max_candidates);
    }

    const float conf_raw_threshold = logit_clamp(conf_threshold);
    double decode_scale_ms[3] = {0.0, 0.0, 0.0};
    size_t decode_scale_candidates[3] = {0, 0, 0};

    const std::array<float *, 3> inputs = {input0, input1, input2};
    for (int s = 0; s < 3; s++)
    {
        const auto ts0 = std::chrono::steady_clock::now();
        const size_t candidate_begin = candidates.size();
        int grid_h = model_in_h / strides[s];
        int grid_w = model_in_w / strides[s];
        int channels = NUM_ANCHORS_PER_SCALE * NUM_CHANNELS_PER_ANCHOR;
        rknn_tensor_attr attr;
        memset(&attr, 0, sizeof(attr));
        if (output_attrs != nullptr && s < output_count)
        {
            attr = output_attrs[s];
            get_output_hwc(attr, grid_h, grid_w, grid_h, grid_w, channels);
        }

        if (grid_h <= 0 || grid_w <= 0 || channels != NUM_ANCHORS_PER_SCALE * NUM_CHANNELS_PER_ANCHOR || inputs[s] == nullptr)
        {
            continue;
        }

        const int stride = strides[s];

        for (int a = 0; a < NUM_ANCHORS_PER_SCALE; ++a)
        {
            const int anchor_idx = anchor_masks[s][a];
            const float anchor_w = anchors[anchor_idx][0];
            const float anchor_h = anchors[anchor_idx][1];

            for (int i = 0; i < grid_h; ++i)
            {
                const float grid_y = (float)i * stride;

                for (int j = 0; j < grid_w; ++j)
                {
                    const float conf_logit = read_output_value(inputs[s], attr, a, CONF_INDEX, i, j, grid_h, grid_w);
                    if (conf_logit < conf_raw_threshold) continue;

                    const float conf = sigmoid(conf_logit);
                    if (conf < conf_threshold) continue;

                    int color_id = 0;
                    float best_color = read_output_value(inputs[s], attr, a, COLOR_START, i, j, grid_h, grid_w);
                    for (int k = 1; k < (COLOR_END - COLOR_START); ++k)
                    {
                        const float color = read_output_value(inputs[s], attr, a, COLOR_START + k, i, j, grid_h, grid_w);
                        if (color > best_color)
                        {
                            best_color = color;
                            color_id = k;
                        }
                    }
                    if (color_id == 2 || color_id == 3) continue;
                    if (detect_color >= 0 && color_id != detect_color) continue;

                    int num_id = 0;
                    float best_num = read_output_value(inputs[s], attr, a, NUM_START, i, j, grid_h, grid_w);
                    for (int k = 1; k < (NUM_END - NUM_START); ++k)
                    {
                        const float num = read_output_value(inputs[s], attr, a, NUM_START + k, i, j, grid_h, grid_w);
                        if (num > best_num)
                        {
                            best_num = num;
                            num_id = k;
                        }
                    }
                    if (best_num <= conf_threshold) continue;

                    Candidate cand;
                    cand.conf = conf;
                    cand.color_id = color_id;
                    cand.num_id = num_id;
                    cand.score_color = best_color;
                    cand.score_num = best_num;

                    const float grid_x = (float)j * stride;
                    float xmin = 0.0f, xmax = 0.0f, ymin = 0.0f, ymax = 0.0f;
                    for (int kp = 0; kp < NUM_KEYPOINTS; ++kp)
                    {
                        const float raw_x = read_output_value(inputs[s], attr, a, KPT_START + kp * 2, i, j, grid_h, grid_w);
                        const float raw_y = read_output_value(inputs[s], attr, a, KPT_START + kp * 2 + 1, i, j, grid_h, grid_w);
                        const float x = raw_x * anchor_w + grid_x;
                        const float y = raw_y * anchor_h + grid_y;
                        cand.kpts[kp * 2] = x;
                        cand.kpts[kp * 2 + 1] = y;

                        if (kp == 0)
                        {
                            xmin = xmax = x;
                            ymin = ymax = y;
                        }
                        else
                        {
                            if (x < xmin) xmin = x;
                            if (x > xmax) xmax = x;
                            if (y < ymin) ymin = y;
                            if (y > ymax) ymax = y;
                        }
                    }

                    cand.bbox[0] = xmin;
                    cand.bbox[1] = ymin;
                    cand.bbox[2] = xmax;
                    cand.bbox[3] = ymax;
                    candidates.push_back(cand);
                }
            }
        }

        const auto ts1 = std::chrono::steady_clock::now();
        decode_scale_ms[s] = std::chrono::duration<double, std::milli>(ts1 - ts0).count();
        decode_scale_candidates[s] = candidates.size() - candidate_begin;
    }

    const auto t1 = std::chrono::steady_clock::now();

    if (candidates.empty())
    {
        // printf("[post_timing] decode_s8=%.3fms decode_s16=%.3fms decode_s32=%.3fms decode=%.3fms nms=0.000ms build=0.000ms total=%.3fms cand_s8=%zu cand_s16=%zu cand_s32=%zu candidates=0 keep=0 out=0\n",
        //        decode_scale_ms[0],
        //        decode_scale_ms[1],
        //        decode_scale_ms[2],
        //        std::chrono::duration<double, std::milli>(t1 - t0).count(),
        //        std::chrono::duration<double, std::milli>(t1 - t0).count(),
        //        decode_scale_candidates[0],
        //        decode_scale_candidates[1],
        //        decode_scale_candidates[2]);
        return 0;
    }

    nms_candidates(candidates, nms_threshold, keep);
    const auto t2 = std::chrono::steady_clock::now();

    // ---- Step 8: Build output structs ----
    int count = 0;
    for (int ki = 0; ki < (int)keep.size() && count < OBJ_NUMB_MAX_SIZE; ki++)
    {
        const int idx = keep[ki];
        const Candidate &c = candidates[idx];

        detect_object_t &out = group->results[count];

        // Bbox (xyxy) — apply padding offset and scale back to original image
        out.bbox_xyxy[0] = (c.bbox[0] - pads.left) / scale_w;
        out.bbox_xyxy[1] = (c.bbox[1] - pads.top) / scale_h;
        out.bbox_xyxy[2] = (c.bbox[2] - pads.left) / scale_w;
        out.bbox_xyxy[3] = (c.bbox[3] - pads.top) / scale_h;

        out.prob = c.conf;
        out.score_num = c.score_num;
        out.score_color = c.score_color;
        out.color_id = c.color_id;
        out.label = c.num_id;

        // Keypoints (scaled back to original image coords)
        out.keypoints[0].x = (c.kpts[0] - pads.left) / scale_w;
        out.keypoints[0].y = (c.kpts[1] - pads.top) / scale_h;
        out.keypoints[1].x = (c.kpts[6] - pads.left) / scale_w;
        out.keypoints[1].y = (c.kpts[7] - pads.top) / scale_h;
        out.keypoints[2].x = (c.kpts[4] - pads.left) / scale_w;
        out.keypoints[2].y = (c.kpts[5] - pads.top) / scale_h;
        out.keypoints[3].x = (c.kpts[2] - pads.left) / scale_w;
        out.keypoints[3].y = (c.kpts[3] - pads.top) / scale_h;

        // Geometry: center, wh, angle from first two keypoints
        float cx = (out.bbox_xyxy[0] + out.bbox_xyxy[2]) / 2.0f;
        float cy = (out.bbox_xyxy[1] + out.bbox_xyxy[3]) / 2.0f;
        float w  = out.bbox_xyxy[2] - out.bbox_xyxy[0];
        float h  = out.bbox_xyxy[3] - out.bbox_xyxy[1];

        // Angle from keypoint 0 -> keypoint 1 (same as Python)
        float dx = out.keypoints[1].x - out.keypoints[0].x;
        float dy = out.keypoints[1].y - out.keypoints[0].y;
        float angle_deg = atan2f(dy, dx) * 180.0f / (float)M_PI;

        out.geometry.center_x  = cx;
        out.geometry.center_y  = cy;
        out.geometry.width     = w;
        out.geometry.height    = h;
        out.geometry.angle_deg = angle_deg;

        count++;
    }

    group->count = count;
    const auto t3 = std::chrono::steady_clock::now();
        // printf("[post_timing] decode_s8=%.3fms decode_s16=%.3fms decode_s32=%.3fms decode=%.3fms nms=%.3fms build=%.3fms total=%.3fms cand_s8=%zu cand_s16=%zu cand_s32=%zu candidates=%zu keep=%zu out=%d\n",
        //     decode_scale_ms[0],
        //     decode_scale_ms[1],
        //     decode_scale_ms[2],
        //    std::chrono::duration<double, std::milli>(t1 - t0).count(),
        //    std::chrono::duration<double, std::milli>(t2 - t1).count(),
        //    std::chrono::duration<double, std::milli>(t3 - t2).count(),
        //    std::chrono::duration<double, std::milli>(t3 - t0).count(),
        //     decode_scale_candidates[0],
        //     decode_scale_candidates[1],
        //     decode_scale_candidates[2],
        //    candidates.size(),
        //    keep.size(),
        //    count);
    return 0;
}

void deinitPostProcess()
{
    // No label files to free for this custom model
}
