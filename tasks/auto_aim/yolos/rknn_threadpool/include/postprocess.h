#ifndef _RKNN_YOLOV5_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV5_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include <cmath>
#include "rknn_api.h"

// -------------------------------------------------------
// Model output layout (matches inference_example.py):
//   Channels 0~7:  keypoints (4 points => 8 values)
//   Channel  8:    confidence (needs sigmoid)
//   Channels 9~12: color scores (4 classes)
//   Channels 13~21: number/digit scores (9 classes)
// -------------------------------------------------------
#define NUM_CHANNELS_PER_ANCHOR 22
#define NUM_KEYPOINTS           4
#define KPT_START               0
#define CONF_INDEX              8
#define COLOR_START             9
#define COLOR_END               13
#define NUM_START               13
#define NUM_END                 22
#define NUM_ANCHORS_PER_SCALE   3

#define OBJ_NUMB_MAX_SIZE       128
#define NMS_THRESH              0.45f
#define CONF_THRESH             0.65f

// Padding rect for letterbox
typedef struct _BOX_RECT
{
    int left;
    int right;
    int top;
    int bottom;
} BOX_RECT;

// Single keypoint
typedef struct _KeyPoint
{
    float x;
    float y;
} KeyPoint;

// Geometry info derived from keypoints
typedef struct _Geometry
{
    float center_x;
    float center_y;
    float width;
    float height;
    float angle_deg;
} Geometry;

// Single detection result (matches Python postprocess_single_head output)
typedef struct _detect_object_t
{
    float bbox_xyxy[4];              // [x1, y1, x2, y2]
    float prob;                      // sigmoid confidence
    float score_num;                 // max number/digit score
    float score_color;               // max color score
    int   color_id;                  // argmax of color scores (0=blue, 1=red, 2/3=filtered)
    int   label;                     // argmax of number/digit scores
    KeyPoint keypoints[NUM_KEYPOINTS]; // 4 keypoints
    Geometry geometry;               // center, wh, angle
} detect_object_t;

// Detection result group
typedef struct _detect_result_group_t
{
    int id;
    int count;
    detect_object_t results[OBJ_NUMB_MAX_SIZE];
} detect_result_group_t;

/**
 * Post-process 3-head float32 RKNN output into detection results.
 *
 * Steps (matching inference_example.py):
 *   1) Decode 3-branch output [1,66,H,W] / [1,H,W,66]
 *   2) Decode keypoints from anchor grids
 *   3) Sigmoid on confidence channel
 *   4) Confidence threshold filter
 *   5) Color / number argmax
 *   6) Color filter (drop color_id 2,3; optionally filter by detect_color)
 *   7) Bbox from keypoints (min/max)
 *   8) NMS on score_num
 *   9) Build output structs with keypoints + geometry
 *
 * @param detect_color  -1 for all colors, 0 for blue-only, 1 for red-only
 */
int post_process(float *input0, float *input1, float *input2,
                 const rknn_tensor_attr *output_attrs, int output_count,
                 int model_in_h, int model_in_w,
                 float conf_threshold, float nms_threshold,
                 BOX_RECT pads, float scale_w, float scale_h,
                 detect_result_group_t *group,
                 int detect_color = -1);

void deinitPostProcess();

#endif //_RKNN_YOLOV5_DEMO_POSTPROCESS_H_
