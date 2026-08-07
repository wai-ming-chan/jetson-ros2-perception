// Copyright 2026 wai-ming
// SPDX-License-Identifier: Apache-2.0

// Unit tests for YOLOv8 postprocessing. These run on CI (x86, no GPU) and on-target.
//
// The planar indexing and letterbox inverse are exactly the kind of arithmetic that a
// typo silently ruins: boxes appear, look plausible, and are all a few pixels off. These
// tests pin the math with hand-computed values.

#include <gtest/gtest.h>

#include <vector>

#include "trt_detector/postprocess.hpp"

using trt_detector::Detection;
using trt_detector::decode_yolov8;
using trt_detector::nms;

namespace
{

// Build a planar tensor: (4 + num_classes) channels x num_anchors.
std::vector<float> make_tensor(int num_classes, int num_anchors)
{
  return std::vector<float>((4 + num_classes) * num_anchors, 0.0f);
}

void set(std::vector<float> & t, int num_anchors, int channel, int anchor, float v)
{
  t[channel * num_anchors + anchor] = v;
}

}  // namespace

TEST(Decode, PlanarLayoutScoreThresholdAndClass)
{
  const int nc = 2, na = 3;
  auto t = make_tensor(nc, na);
  // anchor 0: box centred (100,80) size 40x20, class 0 at 0.9
  set(t, na, 0, 0, 100); set(t, na, 1, 0, 80);
  set(t, na, 2, 0, 40);  set(t, na, 3, 0, 20);
  set(t, na, 4, 0, 0.9f);
  // anchor 1: below threshold
  set(t, na, 0, 1, 300); set(t, na, 1, 1, 300);
  set(t, na, 2, 1, 10);  set(t, na, 3, 1, 10);
  set(t, na, 5, 1, 0.2f);
  // anchor 2: class 1 at 0.6
  set(t, na, 0, 2, 500); set(t, na, 1, 2, 400);
  set(t, na, 2, 2, 60);  set(t, na, 3, 2, 30);
  set(t, na, 5, 2, 0.6f);

  auto dets = decode_yolov8(t.data(), nc, na, 0.5f, 1.0f, 0, 0, 640, 640);
  ASSERT_EQ(dets.size(), 2u);

  EXPECT_EQ(dets[0].class_id, 0);
  EXPECT_FLOAT_EQ(dets[0].score, 0.9f);
  EXPECT_FLOAT_EQ(dets[0].box.x, 80.0f);   // 100 - 40/2
  EXPECT_FLOAT_EQ(dets[0].box.y, 70.0f);   // 80 - 20/2
  EXPECT_FLOAT_EQ(dets[0].box.width, 40.0f);
  EXPECT_FLOAT_EQ(dets[0].box.height, 20.0f);

  EXPECT_EQ(dets[1].class_id, 1);
  EXPECT_FLOAT_EQ(dets[1].score, 0.6f);
}

TEST(Decode, LetterboxInverseFor1080pIn640Square)
{
  // The production case: 1920x1080 letterboxed into 640x640 -> scale 1/3, pad_y 140.
  const int nc = 1, na = 1;
  auto t = make_tensor(nc, na);
  set(t, na, 0, 0, 320);   // cx at network centre
  set(t, na, 1, 0, 320);   // cy at network centre
  set(t, na, 2, 0, 30);
  set(t, na, 3, 0, 30);
  set(t, na, 4, 0, 0.8f);

  const float scale = 640.0f / 1920.0f;   // 1/3
  auto dets = decode_yolov8(t.data(), nc, na, 0.5f, scale, 0, 140, 1920, 1080);
  ASSERT_EQ(dets.size(), 1u);
  // x = (320 - 15 - 0) * 3 = 915 ; y = (320 - 15 - 140) * 3 = 495 ; size 90x90
  EXPECT_NEAR(dets[0].box.x, 915.0f, 1e-3);
  EXPECT_NEAR(dets[0].box.y, 495.0f, 1e-3);
  EXPECT_NEAR(dets[0].box.width, 90.0f, 1e-3);
  EXPECT_NEAR(dets[0].box.height, 90.0f, 1e-3);
}

TEST(Decode, ClampsToImageAndDropsZeroArea)
{
  const int nc = 1, na = 2;
  auto t = make_tensor(nc, na);
  // anchor 0: hangs off the right edge -> clamped
  set(t, na, 0, 0, 630); set(t, na, 1, 0, 100);
  set(t, na, 2, 0, 40);  set(t, na, 3, 0, 40);
  set(t, na, 4, 0, 0.9f);
  // anchor 1: entirely outside -> zero area after clamp -> dropped
  set(t, na, 0, 1, 1000); set(t, na, 1, 1, 1000);
  set(t, na, 2, 1, 20);   set(t, na, 3, 1, 20);
  set(t, na, 4, 1, 0.9f);

  auto dets = decode_yolov8(t.data(), nc, na, 0.5f, 1.0f, 0, 0, 640, 640);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_FLOAT_EQ(dets[0].box.x + dets[0].box.width, 640.0f);
}

TEST(Nms, SuppressesSameClassKeepsOtherClassAndDisjoint)
{
  const cv::Rect2f a(100, 100, 50, 50);
  std::vector<Detection> in = {
    {a, 0.8f, 0},                          // same box, same class, lower score
    {a, 0.9f, 0},                          // winner
    {a, 0.7f, 1},                          // same box, DIFFERENT class -> kept
    {{400, 400, 50, 50}, 0.6f, 0},         // disjoint, same class -> kept
  };
  auto out = nms(in, 0.45f);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_FLOAT_EQ(out[0].score, 0.9f);     // sorted by score
  EXPECT_EQ(out[0].class_id, 0);
  EXPECT_EQ(out[1].class_id, 1);
  EXPECT_FLOAT_EQ(out[2].score, 0.6f);
}
