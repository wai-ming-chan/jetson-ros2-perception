// YOLOv8 postprocessing, split from the node so it can be unit-tested without TensorRT.
//
// The node's hard-to-eyeball logic lives here: the planar output-tensor indexing
// (out[channel * num_anchors + anchor]), the letterbox inverse that maps network
// coordinates back to image pixels, and NMS. CI runners have no GPU and no NvInfer.h,
// but they can and do assert on all of this.

#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace trt_detector
{

struct Detection
{
  cv::Rect2f box;      // in original image coordinates
  float score;
  int class_id;
};

// Decode a YOLOv8 output tensor laid out planar: channel c of anchor i at
// out[c * num_anchors + i], channels = 4 box coords (cx,cy,w,h) then one score per
// class. Applies the score threshold, undoes the letterbox (scale, pad_x, pad_y), and
// clamps boxes to the image; zero-area boxes are dropped.
std::vector<Detection> decode_yolov8(
  const float * out, int num_classes, int num_anchors, float score_threshold,
  float scale, int pad_x, int pad_y, int img_w, int img_h);

// Greedy class-aware non-maximum suppression. Input order does not matter; output is
// sorted by descending score.
std::vector<Detection> nms(std::vector<Detection> dets, float iou_threshold);

}  // namespace trt_detector
