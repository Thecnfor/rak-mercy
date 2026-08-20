#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace deyes_capture_cpp
{

struct FrameSnapshot
{
  std::vector<uint8_t> data;
  std::string encoding;
  int width{0};
  int height{0};
  std::size_t step{0};
  double stamp_sec{0.0};
  uint64_t seq{0};
};

class CaptureWorker
{
public:
  CaptureWorker(
    const std::string & name,
    int sensor_id,
    int width,
    int height,
    int fps,
    std::size_t history_size,
    const std::string & output_encoding,
    bool rotate_180,
    bool mirror_horizontal,
    rclcpp::Logger logger);
  ~CaptureWorker();

  bool start(std::string & error_message);
  void stop();
  std::optional<FrameSnapshot> latest() const;
  std::deque<FrameSnapshot> recent_frames() const;
  double capture_rate_hz() const;
  uint64_t total_failures() const;

private:
  void capture_loop();
  std::string build_pipeline() const;
  static std::string gst_format_for_output(const std::string & output_encoding);

  std::string name_;
  int sensor_id_;
  int width_;
  int height_;
  int fps_;
  std::string output_encoding_;
  bool rotate_180_{false};
  bool mirror_horizontal_{false};
  rclcpp::Logger logger_;

  GstElement * pipeline_{nullptr};
  GstElement * appsink_{nullptr};

  mutable std::mutex mutex_;
  std::optional<FrameSnapshot> latest_;
  std::deque<FrameSnapshot> recent_frames_;
  std::deque<double> receipt_history_;
  std::size_t history_size_{8};
  std::atomic<uint64_t> total_failures_{0};
  std::atomic<bool> stop_requested_{false};
  std::thread thread_;
  uint64_t frame_count_{0};
};

class Imx219StereoCaptureNode : public rclcpp::Node
{
public:
  explicit Imx219StereoCaptureNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Imx219StereoCaptureNode() override;

private:
  sensor_msgs::msg::CameraInfo load_camera_info(
    const std::string & calib_path,
    const std::string & side,
    const std::string & frame_id,
    int output_width,
    int output_height) const;

  sensor_msgs::msg::CameraInfo camera_info_with_stamp(
    const sensor_msgs::msg::CameraInfo & templ,
    double stamp_sec) const;

  sensor_msgs::msg::Image image_msg_from_frame(
    const FrameSnapshot & frame,
    double stamp_sec,
    const std::string & frame_id) const;

  void on_timer();
  void maybe_log_stats(double now_sec);
  double publish_rate_hz() const;
  std::string skew_summary() const;

  std::unique_ptr<CaptureWorker> left_worker_;
  std::unique_ptr<CaptureWorker> right_worker_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::CameraInfo left_info_template_;
  sensor_msgs::msg::CameraInfo right_info_template_;

  std::string left_frame_id_;
  std::string right_frame_id_;
  bool publish_info_{true};
  bool has_camera_info_{false};
  bool reuse_latest_frame_{false};
  bool swap_left_right_{false};
  double pair_max_skew_sec_{0.02};
  double frame_stale_sec_{0.2};
  double publish_period_sec_{1.0 / 30.0};
  double log_stats_period_sec_{2.0};
  double last_stats_log_sec_{0.0};
  double last_skew_ms_{0.0};
  double last_publish_duration_ms_{0.0};
  uint64_t last_pair_left_seq_{0};
  uint64_t last_pair_right_seq_{0};
  uint64_t publish_count_{0};
  uint64_t dropped_skew_count_{0};
  uint64_t dropped_stale_count_{0};
  uint64_t waiting_for_pair_count_{0};
  std::deque<double> publish_history_;
  std::deque<double> skew_history_ms_;
  uint64_t last_logged_publish_count_{0};
  uint64_t last_logged_dropped_skew_count_{0};
  uint64_t last_logged_dropped_stale_count_{0};
  uint64_t last_logged_waiting_for_pair_count_{0};
};

}  // namespace deyes_capture_cpp
