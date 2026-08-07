// TensorRT object detection node for the Jetson perception stack.
//
// Consumes raw bgr8 frames from the camera driver, runs a YOLOv8 engine, and publishes
// vision_msgs/Detection2DArray plus a compressed debug overlay the operator console can
// display directly (point its image_topic at ~/overlay).
//
// Design notes, each earned elsewhere in this repo:
//  - Newest-frame-wins: the subscriber only stores the latest message; a worker thread
//    processes at whatever rate inference allows and stale frames are dropped, never
//    queued. Queues convert throughput into latency (see the console's Issue 8).
//  - CPU preprocessing, measured: trtexec showed H2D+D2H under 0.8 ms total, so a CUDA
//    preprocessing kernel is unlikely to pay for itself. Per-stage timing below proves or
//    refutes that on real data instead of assuming.
//  - The overlay is encoded only when someone subscribes, like the camera's lazy
//    compressed transport: JPEG encoding for nobody is CPU spent on nothing.
//
// Engine expectations (verified for the committed export path):
//   input  "images"  1x3x640x640 float32, RGB, 0..1
//   output "output0" 1x84x8400  float32 -- 4 box coords (cx,cy,w,h) + 80 COCO scores,
//                                planar layout: out[channel * 8400 + anchor]

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>

#include "trt_detector/postprocess.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace trt_detector
{

constexpr int kInputW = 640;
constexpr int kInputH = 640;
constexpr int kNumClasses = 80;
constexpr int kNumAnchors = 8400;

const char * const kCocoNames[kNumClasses] = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
  "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
  "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
  "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
  "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
  "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
  "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
  "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
  "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
  "toothbrush"};

class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      RCLCPP_WARN(rclcpp::get_logger("tensorrt"), "%s", msg);
    }
  }
};

class TrtDetectorNode : public rclcpp::Node
{
public:
  TrtDetectorNode()
  : Node("trt_detector")
  {
    engine_path_ = declare_parameter<std::string>("engine_path", "/models/yolov8n_fp16.engine");
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw");
    score_threshold_ = declare_parameter<double>("score_threshold", 0.35);
    nms_iou_ = declare_parameter<double>("nms_iou_threshold", 0.45);
    overlay_quality_ = declare_parameter<int>("overlay_jpeg_quality", 70);
    // The overlay is a debug view, not a data product. Encoding it at full 1080p was
    // measured at 33.4 ms per frame -- more than inference -- and dropped the detector
    // from 29 Hz to 17.7 Hz. Downscaling first costs ~1 ms and restores the rate.
    overlay_width_ = declare_parameter<int>("overlay_width", 960);
    report_interval_s_ = declare_parameter<double>("rate_report_interval", 5.0);

    load_engine();

    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("~/detections", 10);
    overlay_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("~/overlay/compressed", 1);
    rate_pub_ = create_publisher<std_msgs::msg::Float32>("~/publish_rate", 10);
    latency_pub_ = create_publisher<std_msgs::msg::Float32>("~/latency_ms", 10);

    // Best-effort keep-last-1: drop rather than queue. The callback stores a pointer,
    // nothing more -- all work happens on the worker thread.
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lk(frame_mutex_);
          latest_frame_ = std::move(msg);
        }
        frame_cv_.notify_one();
      });

    worker_ = std::thread(&TrtDetectorNode::worker_loop, this);
    RCLCPP_INFO(
      get_logger(), "detector ready: %s on %s (score>=%.2f, nms=%.2f)",
      engine_path_.c_str(), image_topic_.c_str(), score_threshold_, nms_iou_);
  }

  ~TrtDetectorNode() override
  {
    running_ = false;
    frame_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    for (void * p : bindings_) {
      if (p) {cudaFree(p);}
    }
    if (stream_) {cudaStreamDestroy(stream_);}
    cudaFreeHost(host_input_);
    cudaFreeHost(host_output_);
  }

private:
  void load_engine()
  {
    std::ifstream file(engine_path_, std::ios::binary);
    if (!file) {
      throw std::runtime_error(
              "cannot open engine '" + engine_path_ +
              "' -- engines are device-specific and never shipped; build one with: "
              "trtexec --onnx=yolov8n.onnx --fp16 --saveEngine=" + engine_path_);
    }
    std::vector<char> blob(
      (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
    engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
    if (!engine_) {
      throw std::runtime_error("engine deserialisation failed -- wrong TensorRT version?");
    }
    context_.reset(engine_->createExecutionContext());

    input_index_ = engine_->getBindingIndex("images");
    output_index_ = engine_->getBindingIndex("output0");
    if (input_index_ < 0 || output_index_ < 0) {
      throw std::runtime_error("expected bindings 'images'/'output0' not found in engine");
    }

    input_bytes_ = 3UL * kInputW * kInputH * sizeof(float);
    output_bytes_ = static_cast<size_t>((kNumClasses + 4)) * kNumAnchors * sizeof(float);

    bindings_.assign(engine_->getNbBindings(), nullptr);
    if (cudaMalloc(&bindings_[input_index_], input_bytes_) != cudaSuccess ||
      cudaMalloc(&bindings_[output_index_], output_bytes_) != cudaSuccess ||
      cudaMallocHost(reinterpret_cast<void **>(&host_input_), input_bytes_) != cudaSuccess ||
      cudaMallocHost(reinterpret_cast<void **>(&host_output_), output_bytes_) != cudaSuccess ||
      cudaStreamCreate(&stream_) != cudaSuccess)
    {
      throw std::runtime_error("CUDA allocation failed");
    }
    RCLCPP_INFO(get_logger(), "engine loaded: %zu byte input, %zu byte output",
      input_bytes_, output_bytes_);
  }

  void worker_loop()
  {
    while (running_) {
      sensor_msgs::msg::Image::ConstSharedPtr frame;
      {
        std::unique_lock<std::mutex> lk(frame_mutex_);
        frame_cv_.wait(lk, [this] {return latest_frame_ || !running_;});
        if (!running_) {return;}
        frame = std::move(latest_frame_);
        latest_frame_.reset();
      }
      if (frame->encoding != "bgr8") {
        RCLCPP_WARN_ONCE(
          get_logger(), "unsupported encoding '%s' (need bgr8)", frame->encoding.c_str());
        continue;
      }
      process(*frame);
    }
  }

  void process(const sensor_msgs::msg::Image & msg)
  {
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();

    // Zero-copy view of the ROS message.
    const cv::Mat image(
      msg.height, msg.width, CV_8UC3,
      const_cast<uint8_t *>(msg.data.data()), msg.step);

    // ---- preprocess: letterbox + BGR->RGB + HWC->CHW + /255 into pinned memory ------
    const float scale = std::min(
      static_cast<float>(kInputW) / image.cols, static_cast<float>(kInputH) / image.rows);
    const int scaled_w = static_cast<int>(image.cols * scale);
    const int scaled_h = static_cast<int>(image.rows * scale);
    const int pad_x = (kInputW - scaled_w) / 2;
    const int pad_y = (kInputH - scaled_h) / 2;

    letterbox_.create(kInputH, kInputW, CV_8UC3);
    letterbox_.setTo(cv::Scalar(114, 114, 114));
    cv::resize(
      image, letterbox_(cv::Rect(pad_x, pad_y, scaled_w, scaled_h)),
      {scaled_w, scaled_h}, 0, 0, cv::INTER_LINEAR);

    // Planar float RGB directly into the pinned host buffer.
    const size_t plane = static_cast<size_t>(kInputW) * kInputH;
    cv::Mat planes[3] = {
      cv::Mat(kInputH, kInputW, CV_32F, host_input_ + 2 * plane),   // B -> plane 2
      cv::Mat(kInputH, kInputW, CV_32F, host_input_ + 1 * plane),   // G -> plane 1
      cv::Mat(kInputH, kInputW, CV_32F, host_input_ + 0 * plane)};  // R -> plane 0
    cv::split(letterbox_, split_u8_);
    for (int c = 0; c < 3; ++c) {
      split_u8_[c].convertTo(planes[c], CV_32F, 1.0 / 255.0);
    }
    const auto t1 = clock::now();

    // ---- inference (H2D + execute + D2H) --------------------------------------------
    cudaMemcpyAsync(
      bindings_[input_index_], host_input_, input_bytes_, cudaMemcpyHostToDevice, stream_);
    context_->enqueueV2(bindings_.data(), stream_, nullptr);
    cudaMemcpyAsync(
      host_output_, bindings_[output_index_], output_bytes_, cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);
    const auto t2 = clock::now();

    // ---- decode + NMS (unit-tested in test/test_postprocess.cpp) --------------------
    std::vector<Detection> dets = nms(
      decode_yolov8(
        host_output_, kNumClasses, kNumAnchors,
        static_cast<float>(score_threshold_), scale, pad_x, pad_y,
        image.cols, image.rows),
      static_cast<float>(nms_iou_));
    const auto t3 = clock::now();

    publish_detections(msg.header, dets);
    publish_overlay(msg.header, image, dets);
    const auto t4 = clock::now();

    record_timing(t0, t1, t2, t3, t4);
  }

  void publish_detections(
    const std_msgs::msg::Header & header, const std::vector<Detection> & dets)
  {
    vision_msgs::msg::Detection2DArray out;
    out.header = header;
    for (const auto & d : dets) {
      vision_msgs::msg::Detection2D det;
      det.header = header;
      det.bbox.center.position.x = d.box.x + d.box.width / 2;
      det.bbox.center.position.y = d.box.y + d.box.height / 2;
      det.bbox.size_x = d.box.width;
      det.bbox.size_y = d.box.height;
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = kCocoNames[d.class_id];
      hyp.hypothesis.score = d.score;
      det.results.push_back(hyp);
      out.detections.push_back(det);
    }
    detections_pub_->publish(out);
  }

  void publish_overlay(
    const std_msgs::msg::Header & header, const cv::Mat & image,
    const std::vector<Detection> & dets)
  {
    // JPEG-encoding a 1080p frame for nobody is ~10 ms of CPU per frame wasted; skip
    // unless the topic actually has subscribers (the console, typically).
    if (overlay_pub_->get_subscription_count() == 0) {
      return;
    }
    // Downscale BEFORE drawing: fewer pixels to annotate and, far more importantly,
    // fewer to JPEG-encode. Boxes are scaled to match.
    float s = 1.0f;
    if (overlay_width_ > 0 && image.cols > overlay_width_) {
      s = static_cast<float>(overlay_width_) / image.cols;
      cv::resize(image, overlay_, {}, s, s, cv::INTER_AREA);
    } else {
      image.copyTo(overlay_);
    }
    for (const auto & d : dets) {
      const cv::Rect r(
        cv::Rect2f(d.box.x * s, d.box.y * s, d.box.width * s, d.box.height * s));
      cv::rectangle(overlay_, r, {80, 220, 60}, 2);
      char label[64];
      std::snprintf(
        label, sizeof(label), "%s %.2f", kCocoNames[d.class_id], d.score);
      const int y = std::max(r.y - 6, 12);
      cv::putText(
        overlay_, label, {r.x, y}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 0}, 3);
      cv::putText(
        overlay_, label, {r.x, y}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {80, 220, 60}, 1);
    }
    auto msg = sensor_msgs::msg::CompressedImage();
    msg.header = header;
    msg.format = "jpeg";
    cv::imencode(
      ".jpg", overlay_, msg.data, {cv::IMWRITE_JPEG_QUALITY, overlay_quality_});
    overlay_pub_->publish(std::move(msg));
  }

  template<typename T>
  void record_timing(T t0, T t1, T t2, T t3, T t4)
  {
    const auto ms = [](T a, T b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
      };
    pre_ms_.push_back(ms(t0, t1));
    infer_ms_.push_back(ms(t1, t2));
    post_ms_.push_back(ms(t2, t3));
    out_ms_.push_back(ms(t3, t4));
    ++frames_;

    const auto now = std::chrono::steady_clock::now();
    if (last_report_.time_since_epoch().count() == 0) {
      last_report_ = now;
    }
    const double elapsed = std::chrono::duration<double>(now - last_report_).count();
    if (elapsed < report_interval_s_) {
      return;
    }
    const auto med = [](std::vector<double> & v) {
        if (v.empty()) {return 0.0;}
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
      };
    const double total =
      med(pre_ms_) + med(infer_ms_) + med(post_ms_) + med(out_ms_);
    RCLCPP_INFO(
      get_logger(),
      "detect: %.1f Hz | pre %.1f + infer %.1f + post %.1f + out %.1f = %.1f ms (medians)",
      frames_ / elapsed, med(pre_ms_), med(infer_ms_), med(post_ms_), med(out_ms_), total);

    std_msgs::msg::Float32 rate;
    rate.data = static_cast<float>(frames_ / elapsed);
    rate_pub_->publish(rate);
    std_msgs::msg::Float32 lat;
    lat.data = static_cast<float>(total);
    latency_pub_->publish(lat);

    pre_ms_.clear();
    infer_ms_.clear();
    post_ms_.clear();
    out_ms_.clear();
    frames_ = 0;
    last_report_ = now;
  }

  // parameters
  std::string engine_path_;
  std::string image_topic_;
  double score_threshold_{0.35};
  double nms_iou_{0.45};
  int overlay_quality_{70};
  int overlay_width_{960};
  double report_interval_s_{5.0};

  // TensorRT
  TrtLogger trt_logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::vector<void *> bindings_;
  int input_index_{-1};
  int output_index_{-1};
  size_t input_bytes_{0};
  size_t output_bytes_{0};
  float * host_input_{nullptr};
  float * host_output_{nullptr};
  cudaStream_t stream_{nullptr};

  // scratch buffers reused across frames to avoid per-frame allocation
  cv::Mat letterbox_;
  cv::Mat overlay_;
  std::vector<cv::Mat> split_u8_;

  // frame handoff
  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_frame_;
  std::atomic<bool> running_{true};
  std::thread worker_;

  // timing
  std::vector<double> pre_ms_, infer_ms_, post_ms_, out_ms_;
  int64_t frames_{0};
  std::chrono::steady_clock::time_point last_report_{};

  // interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr overlay_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rate_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_pub_;
};

}  // namespace trt_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<trt_detector::TrtDetectorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("trt_detector"), "%s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
