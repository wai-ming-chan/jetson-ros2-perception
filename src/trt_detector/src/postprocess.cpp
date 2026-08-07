// Copyright 2026 wai-ming
// SPDX-License-Identifier: Apache-2.0

#include "trt_detector/postprocess.hpp"

#include <algorithm>

namespace trt_detector
{

std::vector<Detection> decode_yolov8(
  const float * out, int num_classes, int num_anchors, float score_threshold,
  float scale, int pad_x, int pad_y, int img_w, int img_h)
{
  std::vector<Detection> raw;
  for (int i = 0; i < num_anchors; ++i) {
    int best = -1;
    float best_score = score_threshold;
    for (int c = 0; c < num_classes; ++c) {
      const float s = out[(4 + c) * num_anchors + i];
      if (s > best_score) {
        best_score = s;
        best = c;
      }
    }
    if (best < 0) {
      continue;
    }
    const float cx = out[0 * num_anchors + i];
    const float cy = out[1 * num_anchors + i];
    const float w = out[2 * num_anchors + i];
    const float h = out[3 * num_anchors + i];
    const float x = (cx - w / 2 - pad_x) / scale;
    const float y = (cy - h / 2 - pad_y) / scale;
    cv::Rect2f box(x, y, w / scale, h / scale);
    box &= cv::Rect2f(0, 0, static_cast<float>(img_w), static_cast<float>(img_h));
    if (box.area() > 0) {
      raw.push_back({box, best_score, best});
    }
  }
  return raw;
}

std::vector<Detection> nms(std::vector<Detection> dets, float iou_threshold)
{
  std::sort(
    dets.begin(), dets.end(),
    [](const Detection & a, const Detection & b) {return a.score > b.score;});
  std::vector<Detection> kept;
  std::vector<bool> removed(dets.size(), false);
  for (size_t i = 0; i < dets.size(); ++i) {
    if (removed[i]) {
      continue;
    }
    kept.push_back(dets[i]);
    for (size_t j = i + 1; j < dets.size(); ++j) {
      if (removed[j] || dets[j].class_id != dets[i].class_id) {
        continue;
      }
      const float inter = (dets[i].box & dets[j].box).area();
      const float uni = dets[i].box.area() + dets[j].box.area() - inter;
      if (uni > 0 && inter / uni > iou_threshold) {
        removed[j] = true;
      }
    }
  }
  return kept;
}

}  // namespace trt_detector
