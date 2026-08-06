// Argus-backed camera driver for NVIDIA Jetson.
//
// Pulls frames from nvarguscamerasrc through a GStreamer appsink and republishes them as
// sensor_msgs/Image. The hardware ISP performs debayer, auto-exposure and auto-white-
// balance, so this node never sees raw Bayer. That is deliberate: the raw RG10 -> CUDA
// debayer path is the comparison arm of the benchmark matrix and lives in its own node.
//
// Verified sensor modes on this hardware (2-lane CSI, see PROGRESS.md):
//   3840x2160 @ 30 fps
//   1920x1080 @ 60 fps
// There is no 12 MP mode. Requesting one will fail to negotiate.

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <atomic>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace jetson_camera
{

class ArgusCameraNode : public rclcpp::Node
{
public:
  ArgusCameraNode()
  : Node("argus_camera")
  {
    sensor_id_ = declare_parameter<int>("sensor_id", 0);
    width_ = declare_parameter<int>("width", 1920);
    height_ = declare_parameter<int>("height", 1080);
    framerate_ = declare_parameter<int>("framerate", 60);
    flip_method_ = declare_parameter<int>("flip_method", 0);
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_optical_frame");
    exposure_time_range_ = declare_parameter<std::string>("exposure_time_range", "");
    gain_range_ = declare_parameter<std::string>("gain_range", "");
    // Argus auto-exposure needs a few frames to converge; the first frames come out
    // black or as full-scale noise. Dropping them avoids publishing garbage at startup.
    warmup_frames_ = declare_parameter<int>("warmup_frames", 30);
    const auto camera_info_url = declare_parameter<std::string>("camera_info_url", "");

    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, "imx477", camera_info_url);

    publisher_ = image_transport::create_camera_publisher(this, "image_raw");

    start_pipeline();
    capture_thread_ = std::thread(&ArgusCameraNode::capture_loop, this);
  }

  ~ArgusCameraNode() override
  {
    stop();
  }

private:
  std::string build_pipeline_description() const
  {
    std::ostringstream pipeline;
    pipeline << "nvarguscamerasrc sensor-id=" << sensor_id_;
    if (!exposure_time_range_.empty()) {
      pipeline << " exposuretimerange=\"" << exposure_time_range_ << "\"";
    }
    if (!gain_range_.empty()) {
      pipeline << " gainrange=\"" << gain_range_ << "\"";
    }
    // NVMM -> BGRx on the hardware converter, then BGR for ROS. The videoconvert step is
    // a known CPU cost and is one of the things week 2 measures and optimises.
    pipeline
      << " ! video/x-raw(memory:NVMM),width=" << width_ << ",height=" << height_
      << ",framerate=" << framerate_ << "/1"
      << " ! nvvidconv flip-method=" << flip_method_
      << " ! video/x-raw,format=BGRx"
      << " ! videoconvert ! video/x-raw,format=BGR"
      << " ! appsink name=sink sync=false max-buffers=2 drop=true";
    return pipeline.str();
  }

  void start_pipeline()
  {
    const std::string description = build_pipeline_description();
    RCLCPP_INFO(get_logger(), "GStreamer pipeline: %s", description.c_str());

    GError * error = nullptr;
    pipeline_ = gst_parse_launch(description.c_str(), &error);
    if (!pipeline_ || error) {
      const std::string message = error ? error->message : "unknown error";
      if (error) {
        g_error_free(error);
      }
      throw std::runtime_error("failed to build pipeline: " + message);
    }

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!appsink_) {
      throw std::runtime_error("appsink element not found in pipeline");
    }

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      // The usual cause is a missing /tmp/argus_socket mount when running in a container.
      throw std::runtime_error(
              "failed to set pipeline PLAYING -- is /tmp/argus_socket mounted?");
    }
    RCLCPP_INFO(get_logger(), "camera streaming at %dx%d@%d", width_, height_, framerate_);
  }

  void capture_loop()
  {
    int64_t frames_seen = 0;

    while (running_ && rclcpp::ok()) {
      GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink_));
      if (!sample) {
        if (running_) {
          RCLCPP_WARN(get_logger(), "appsink returned no sample; stream ended");
        }
        break;
      }

      if (frames_seen++ < warmup_frames_) {
        gst_sample_unref(sample);
        continue;
      }

      publish(sample);
      gst_sample_unref(sample);
    }
  }

  void publish(GstSample * sample)
  {
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
      return;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      RCLCPP_WARN(get_logger(), "failed to map buffer");
      return;
    }

    auto image = std::make_unique<sensor_msgs::msg::Image>();
    image->header.stamp = now();
    image->header.frame_id = frame_id_;
    image->height = static_cast<uint32_t>(height_);
    image->width = static_cast<uint32_t>(width_);
    image->encoding = "bgr8";
    image->is_bigendian = 0;
    image->step = static_cast<uint32_t>(width_ * 3);
    image->data.assign(map.data, map.data + map.size);

    gst_buffer_unmap(buffer, &map);

    auto camera_info = camera_info_manager_->getCameraInfo();
    camera_info.header = image->header;

    publisher_.publish(*image, camera_info);
  }

  void stop()
  {
    running_ = false;

    if (appsink_) {
      // Unblocks a capture thread parked in gst_app_sink_pull_sample().
      gst_app_sink_set_emit_signals(GST_APP_SINK(appsink_), FALSE);
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (appsink_) {
      gst_object_unref(appsink_);
      appsink_ = nullptr;
    }
    if (pipeline_) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  int sensor_id_{0};
  int width_{1920};
  int height_{1080};
  int framerate_{60};
  int flip_method_{0};
  int warmup_frames_{30};
  std::string frame_id_;
  std::string exposure_time_range_;
  std::string gain_range_;

  GstElement * pipeline_{nullptr};
  GstElement * appsink_{nullptr};
  std::thread capture_thread_;
  std::atomic<bool> running_{true};

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  image_transport::CameraPublisher publisher_;
};

}  // namespace jetson_camera

int main(int argc, char ** argv)
{
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<jetson_camera::ArgusCameraNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("argus_camera"), "%s", e.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
