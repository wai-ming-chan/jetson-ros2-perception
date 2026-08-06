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
#include <chrono>
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
#include <std_msgs/msg/float32.hpp>

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
    // Seconds without a frame before the pipeline is torn down and rebuilt.
    stall_restart_s_ = declare_parameter<int>("stall_restart_sec", 5);
    report_interval_s_ = declare_parameter<double>("rate_report_interval", 5.0);
    // 0 = use all available cores; 1 = GStreamer's single-threaded default.
    convert_threads_ = declare_parameter<int>("convert_threads", 0);
    const auto camera_info_url = declare_parameter<std::string>("camera_info_url", "");

    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, "imx477", camera_info_url);

    publisher_ = image_transport::create_camera_publisher(this, "image_raw");
    // Publish the measured rate so consumers report the PUBLISHER's throughput
    // rather than their own. A Python subscriber deserialising 6 MB frames reads
    // far below line rate and would otherwise misreport the pipeline as slow.
    // "~/" makes this private to the node -> /argus_camera/publish_rate. A bare
    // "publish_rate" would resolve against the NAMESPACE, not the node name, landing
    // at /publish_rate and silently missing any subscriber expecting the node-scoped
    // name.
    rate_publisher_ = create_publisher<std_msgs::msg::Float32>("~/publish_rate", 10);

    start_pipeline();
    capture_thread_ = std::thread(&ArgusCameraNode::capture_loop, this);
  }

  ~ArgusCameraNode() override
  {
    stop();
  }

  bool unrecoverable() const
  {
    return unrecoverable_;
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
    // NVMM -> BGRx on the hardware converter, then BGR for ROS.
    //
    // videoconvert is the one CPU-bound stage here and it defaults to a SINGLE thread.
    // Measured at 15W: 1080p60 runs at full rate using 0.65 core, but 3840x2160@30 reaches
    // only 26.8 Hz with one core pinned at ~74% and the other five idle -- twice the pixel
    // rate, one thread to do it. n-threads=0 lets it use all cores.
    pipeline
      << " ! video/x-raw(memory:NVMM),width=" << width_ << ",height=" << height_
      << ",framerate=" << framerate_ << "/1"
      << " ! nvvidconv flip-method=" << flip_method_
      << " ! video/x-raw,format=BGRx"
      << " ! videoconvert n-threads=" << convert_threads_
      << " ! video/x-raw,format=BGR"
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
    // Deliberately NOT "camera streaming": set_state returns ASYNC, so at this point
    // nothing proves capture works. When a second process held the only Argus session,
    // the old success log printed and then no frame ever arrived.
    RCLCPP_INFO(
      get_logger(), "pipeline PLAYING requested (%dx%d@%d); waiting for first frame",
      width_, height_, framerate_);
  }

  void capture_loop()
  {
    int64_t frames_seen = 0;
    int stalled_seconds = 0;
    int rebuilds_without_frame = 0;
    bool announced = false;

    while (running_ && rclcpp::ok()) {
      // Bounded wait. gst_app_sink_pull_sample() blocks FOREVER when the pipeline wedges
      // (nvargus-daemon dying is a known L4T failure mode), leaving a node that looks
      // alive -- services answer, topics exist -- but never publishes again.
      GstSample * sample =
        gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), GST_SECOND);

      if (sample) {
        stalled_seconds = 0;
        rebuilds_without_frame = 0;
        if (frames_seen++ < warmup_frames_) {
          gst_sample_unref(sample);
          continue;
        }
        if (!announced) {
          announced = true;
          RCLCPP_INFO(get_logger(), "capture confirmed: first frame published");
        }
        publish(sample);
        report_rate();
        gst_sample_unref(sample);
        continue;
      }

      if (!running_ || !rclcpp::ok()) {
        break;
      }

      const bool ended = gst_app_sink_is_eos(GST_APP_SINK(appsink_));
      if (!ended && ++stalled_seconds < stall_restart_s_) {
        continue;
      }

      if (ended) {
        RCLCPP_ERROR(get_logger(), "capture stream ended (EOS); rebuilding pipeline");
      } else {
        RCLCPP_ERROR(
          get_logger(), "no frame for %d s; assuming wedged pipeline, rebuilding",
          stalled_seconds);
      }

      // A rebuild "succeeding" only means PLAYING was accepted -- with Argus held by
      // another process that happens and still no frame ever arrives. Cap consecutive
      // rebuilds that yield nothing, or this would cycle forever.
      if (rebuilds_without_frame >= 3 || !rebuild_pipeline()) {
        break;
      }
      ++rebuilds_without_frame;
      stalled_seconds = 0;
      frames_seen = 0;      // Argus AE reconverges after a restart: drop warmup again
      announced = false;
    }

    // Reaching here while the node should still be running means capture is
    // unrecoverable (nvargus-daemon dead, camera held by another process, ...). The old
    // behaviour was to leave the node up publishing nothing -- a zombie that looks
    // healthy from outside and freezes every consumer. Shut down instead, so a
    // supervisor (launch respawn, systemd, docker restart policy) can act.
    if (running_ && rclcpp::ok()) {
      RCLCPP_FATAL(get_logger(), "capture is unrecoverable; shutting down node");
      unrecoverable_ = true;
      rclcpp::shutdown();
    }
  }

  bool rebuild_pipeline()
  {
    for (int attempt = 1; attempt <= 3 && running_ && rclcpp::ok(); ++attempt) {
      teardown_pipeline();
      std::this_thread::sleep_for(std::chrono::seconds(2));
      try {
        start_pipeline();
        RCLCPP_INFO(get_logger(), "pipeline restarted (attempt %d)", attempt);
        return true;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          get_logger(), "pipeline restart attempt %d failed: %s", attempt, e.what());
      }
    }
    return false;
  }

  void teardown_pipeline()
  {
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
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

  // Publisher-side rate, logged periodically.
  //
  // This exists because `ros2 topic hz` cannot answer the question on its own: it is a
  // Python subscriber deserialising ~6 MB per frame, so a low reading may mean the
  // subscriber cannot keep up rather than that the pipeline is slow. Comparing this
  // number against `ros2 topic hz` separates "we are not producing frames" from "the
  // consumer cannot drink them fast enough".
  void report_rate()
  {
    ++frames_since_report_;

    const rclcpp::Time current = now();
    if (last_report_.nanoseconds() == 0) {
      last_report_ = current;
      return;
    }

    const double elapsed = (current - last_report_).seconds();
    if (elapsed >= report_interval_s_) {
      const double hz = static_cast<double>(frames_since_report_) / elapsed;
      RCLCPP_INFO(
        get_logger(), "publisher: %.2f Hz (%ld frames in %.1f s)",
        hz, frames_since_report_, elapsed);

      std_msgs::msg::Float32 rate_msg;
      rate_msg.data = static_cast<float>(hz);
      rate_publisher_->publish(rate_msg);
      frames_since_report_ = 0;
      last_report_ = current;
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
    // Join first: the capture thread's pull has a one second bound, so it notices
    // running_ promptly, and only this thread ever touches the pipeline afterwards.
    // (The previous version poked the pipeline from here while the capture thread was
    // still using it, and its emit_signals call never unblocked anything anyway --
    // only the state change to NULL did.)
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    teardown_pipeline();
  }

  int sensor_id_{0};
  int width_{1920};
  int height_{1080};
  int framerate_{60};
  int flip_method_{0};
  int warmup_frames_{30};
  int stall_restart_s_{5};
  double report_interval_s_{5.0};
  int convert_threads_{0};
  int64_t frames_since_report_{0};
  rclcpp::Time last_report_{0, 0, RCL_ROS_TIME};
  std::string frame_id_;
  std::string exposure_time_range_;
  std::string gain_range_;

  GstElement * pipeline_{nullptr};
  GstElement * appsink_{nullptr};
  std::thread capture_thread_;
  std::atomic<bool> running_{true};
  std::atomic<bool> unrecoverable_{false};

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  image_transport::CameraPublisher publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rate_publisher_;
};

}  // namespace jetson_camera

int main(int argc, char ** argv)
{
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    auto node = std::make_shared<jetson_camera::ArgusCameraNode>();
    rclcpp::spin(node);
    // Exit non-zero when capture died rather than the user asking us to stop, so
    // Restart=on-failure supervisors distinguish the two.
    if (node->unrecoverable()) {
      exit_code = 1;
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("argus_camera"), "%s", e.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
