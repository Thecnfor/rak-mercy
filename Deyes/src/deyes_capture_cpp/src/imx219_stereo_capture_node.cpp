#include "deyes_capture_cpp/imx219_stereo_capture_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gst/base/gstbasesink.h>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

double now_sec()
{
  using Clock = std::chrono::system_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

double percentile(std::vector<double> values, double ratio)
{
  if (values.empty()) {
    return 0.0;
  }
  ratio = std::clamp(ratio, 0.0, 1.0);
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
    std::floor(ratio * static_cast<double>(values.size() - 1U)));
  return values[index];
}

std::string format_metric(double value, int precision)
{
  if (!std::isfinite(value)) {
    return "nan";
  }
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(precision);
  stream << value;
  return stream.str();
}

template<std::size_t N>
std::array<double, N> parse_flat_array(const YAML::Node & node)
{
  std::array<double, N> values{};
  std::size_t index = 0;
  if (!node || !node.IsSequence()) {
    throw std::runtime_error("YAML matrix node is missing or not a sequence");
  }
  for (const auto & row : node) {
    if (row.IsSequence()) {
      for (const auto & value : row) {
        if (index >= N) {
          throw std::runtime_error("YAML matrix has too many elements");
        }
        values[index++] = value.as<double>();
      }
    } else {
      if (index >= N) {
        throw std::runtime_error("YAML vector has too many elements");
      }
      values[index++] = row.as<double>();
    }
  }
  if (index != N) {
    throw std::runtime_error("YAML matrix has unexpected element count");
  }
  return values;
}

std::vector<double> parse_vector(const YAML::Node & node)
{
  if (!node || !node.IsSequence()) {
    throw std::runtime_error("YAML vector node is missing or not a sequence");
  }
  std::vector<double> values;
  values.reserve(node.size());
  for (const auto & value : node) {
    if (value.IsSequence()) {
      for (const auto & nested : value) {
        values.push_back(nested.as<double>());
      }
    } else {
      values.push_back(value.as<double>());
    }
  }
  return values;
}

}  // namespace

namespace deyes_capture_cpp
{

CaptureWorker::CaptureWorker(
  const std::string & name,
  int sensor_id,
  int width,
  int height,
  int fps,
  std::size_t history_size,
  const std::string & output_encoding,
  bool rotate_180,
  bool mirror_horizontal,
  rclcpp::Logger logger)
: name_(name),
  sensor_id_(sensor_id),
  width_(width),
  height_(height),
  fps_(fps),
  output_encoding_(output_encoding),
  rotate_180_(rotate_180),
  mirror_horizontal_(mirror_horizontal),
  logger_(std::move(logger)),
  history_size_(std::max<std::size_t>(history_size, 2U))
{
  receipt_history_.clear();
  recent_frames_.clear();
}

CaptureWorker::~CaptureWorker()
{
  stop();
}

bool CaptureWorker::start(std::string & error_message)
{
  stop_requested_.store(false);
  GError * error = nullptr;
  const std::string pipeline = build_pipeline();
  pipeline_ = gst_parse_launch(pipeline.c_str(), &error);
  if (pipeline_ == nullptr) {
    error_message = error != nullptr ? error->message : "gst_parse_launch failed";
    if (error != nullptr) {
      g_error_free(error);
    }
    return false;
  }

  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
  if (appsink_ == nullptr) {
    error_message = "failed to locate appsink named 'sink'";
    stop();
    return false;
  }

  gst_app_sink_set_drop(GST_APP_SINK(appsink_), true);
  gst_app_sink_set_max_buffers(GST_APP_SINK(appsink_), 1);
  gst_base_sink_set_sync(GST_BASE_SINK(appsink_), false);

  const GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    error_message = "failed to set GStreamer pipeline to PLAYING";
    stop();
    return false;
  }

  thread_ = std::thread(&CaptureWorker::capture_loop, this);
  return true;
}

void CaptureWorker::stop()
{
  stop_requested_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }

  if (pipeline_ != nullptr) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
  }
  if (appsink_ != nullptr) {
    gst_object_unref(appsink_);
    appsink_ = nullptr;
  }
  if (pipeline_ != nullptr) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
}

std::optional<FrameSnapshot> CaptureWorker::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

std::deque<FrameSnapshot> CaptureWorker::recent_frames() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return recent_frames_;
}

double CaptureWorker::capture_rate_hz() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (receipt_history_.size() < 2) {
    return 0.0;
  }
  const double elapsed = receipt_history_.back() - receipt_history_.front();
  if (elapsed <= 0.0) {
    return 0.0;
  }
  return static_cast<double>(receipt_history_.size() - 1U) / elapsed;
}

uint64_t CaptureWorker::total_failures() const
{
  return total_failures_.load();
}

void CaptureWorker::capture_loop()
{
  while (!stop_requested_.load()) {
    GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 50 * GST_MSECOND);
    if (sample == nullptr) {
      continue;
    }

    GstCaps * caps = gst_sample_get_caps(sample);
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    if (caps == nullptr || buffer == nullptr) {
      total_failures_.fetch_add(1);
      gst_sample_unref(sample);
      continue;
    }

    GstStructure * structure = gst_caps_get_structure(caps, 0);
    if (structure == nullptr) {
      total_failures_.fetch_add(1);
      gst_sample_unref(sample);
      continue;
    }

    gint width = 0;
    gint height = 0;
    const gchar * format = gst_structure_get_string(structure, "format");
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);

    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
      total_failures_.fetch_add(1);
      gst_sample_unref(sample);
      continue;
    }

    FrameSnapshot snapshot;
    snapshot.width = static_cast<int>(width);
    snapshot.height = static_cast<int>(height);
    snapshot.encoding =
      (format != nullptr && std::string(format) == "GRAY8") ? "mono8" : "bgr8";
    snapshot.step = snapshot.height > 0 ?
      (static_cast<std::size_t>(map_info.size) / static_cast<std::size_t>(snapshot.height)) : 0U;
    snapshot.data.resize(map_info.size);
    std::memcpy(snapshot.data.data(), map_info.data, map_info.size);
    snapshot.stamp_sec = now_sec();

    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++frame_count_;
      snapshot.seq = frame_count_;
      latest_ = std::move(snapshot);
      recent_frames_.push_back(*latest_);
      while (recent_frames_.size() > history_size_) {
        recent_frames_.pop_front();
      }
      receipt_history_.push_back(latest_->stamp_sec);
      if (receipt_history_.size() > 120) {
        receipt_history_.pop_front();
      }
    }
  }
}

std::string CaptureWorker::build_pipeline() const
{
  const auto output_format = gst_format_for_output(output_encoding_);
  int flip_method = -1;
  if (rotate_180_ && mirror_horizontal_) {
    // rotate 180 + horizontal mirror == vertical flip
    flip_method = 6;
  } else if (rotate_180_) {
    flip_method = 2;
  } else if (mirror_horizontal_) {
    flip_method = 4;
  }
  const std::string flip_segment =
    flip_method >= 0 ? (" flip-method=" + std::to_string(flip_method)) : "";
  return
    "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id_) + " ! "
    "video/x-raw(memory:NVMM),width=" + std::to_string(width_) +
    ",height=" + std::to_string(height_) +
    ",format=NV12,framerate=" + std::to_string(fps_) + "/1 ! "
    "nvvidconv" + flip_segment + " ! video/x-raw,format=BGRx ! "
    "videoconvert ! video/x-raw,format=" + output_format + " ! "
    "appsink name=sink drop=true max-buffers=1 sync=false";
}

std::string CaptureWorker::gst_format_for_output(const std::string & output_encoding)
{
  if (output_encoding == "mono8") {
    return "GRAY8";
  }
  return "BGR";
}

Imx219StereoCaptureNode::Imx219StereoCaptureNode(const rclcpp::NodeOptions & options)
: Node("imx219_stereo_capture_node", options)
{
  int argc = 0;
  char ** argv = nullptr;
  gst_init(&argc, &argv);

  declare_parameter<int>("left_sensor_id", 0);
  declare_parameter<int>("right_sensor_id", 1);
  declare_parameter<int>("width", 640);
  declare_parameter<int>("height", 360);
  declare_parameter<int>("fps", 30);
  declare_parameter<std::string>("output_encoding", "mono8");
  declare_parameter<std::string>("calib_path", "");
  declare_parameter<std::string>("left_image_topic", "/x1/left_camera/image_raw");
  declare_parameter<std::string>("right_image_topic", "/x1/right_camera/image_raw");
  declare_parameter<std::string>("left_info_topic", "/x1/left_camera/camera_info");
  declare_parameter<std::string>("right_info_topic", "/x1/right_camera/camera_info");
  declare_parameter<std::string>("left_frame_id", "left_camera_optical_frame");
  declare_parameter<std::string>("right_frame_id", "right_camera_optical_frame");
  declare_parameter<bool>("publish_info", true);
  declare_parameter<double>("target_publish_hz", 30.0);
  declare_parameter<double>("pair_max_skew_ms", 20.0);
  declare_parameter<double>("frame_stale_sec", 0.2);
  declare_parameter<double>("log_stats_period_sec", 2.0);
  declare_parameter<int>("history_size", 8);
  declare_parameter<bool>("reuse_latest_frame", false);
  declare_parameter<bool>("rotate_180", true);
  declare_parameter<bool>("mirror_horizontal", true);
  declare_parameter<bool>("swap_left_right", true);

  const int width = get_parameter("width").as_int();
  const int height = get_parameter("height").as_int();
  const int fps = get_parameter("fps").as_int();
  const auto output_encoding = get_parameter("output_encoding").as_string();
  const auto left_sensor_id = get_parameter("left_sensor_id").as_int();
  const auto right_sensor_id = get_parameter("right_sensor_id").as_int();
  const auto history_size = static_cast<std::size_t>(get_parameter("history_size").as_int());
  const auto rotate_180 = get_parameter("rotate_180").as_bool();
  const auto mirror_horizontal = get_parameter("mirror_horizontal").as_bool();
  swap_left_right_ = get_parameter("swap_left_right").as_bool();

  left_frame_id_ = get_parameter("left_frame_id").as_string();
  right_frame_id_ = get_parameter("right_frame_id").as_string();
  publish_info_ = get_parameter("publish_info").as_bool();
  reuse_latest_frame_ = get_parameter("reuse_latest_frame").as_bool();
  pair_max_skew_sec_ = get_parameter("pair_max_skew_ms").as_double() / 1000.0;
  frame_stale_sec_ = get_parameter("frame_stale_sec").as_double();
  log_stats_period_sec_ = get_parameter("log_stats_period_sec").as_double();
  const double target_publish_hz = get_parameter("target_publish_hz").as_double();
  publish_period_sec_ = target_publish_hz > 0.0 ? (1.0 / target_publish_hz) : (1.0 / 30.0);

  left_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
    get_parameter("left_image_topic").as_string(), rclcpp::SensorDataQoS());
  right_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
    get_parameter("right_image_topic").as_string(), rclcpp::SensorDataQoS());
  left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    get_parameter("left_info_topic").as_string(), rclcpp::SensorDataQoS());
  right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    get_parameter("right_info_topic").as_string(), rclcpp::SensorDataQoS());
  pair_diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/x1/stereo/pair_diagnostics", 10);

  const auto calib_path = get_parameter("calib_path").as_string();
  if (publish_info_ && !calib_path.empty()) {
    left_info_template_ = load_camera_info(calib_path, "left", left_frame_id_, width, height);
    right_info_template_ = load_camera_info(calib_path, "right", right_frame_id_, width, height);
    has_camera_info_ = true;
  }

  left_worker_ = std::make_unique<CaptureWorker>(
    "left", left_sensor_id, width, height, fps, history_size, output_encoding, rotate_180,
    mirror_horizontal,
    get_logger());
  right_worker_ = std::make_unique<CaptureWorker>(
    "right", right_sensor_id, width, height, fps, history_size, output_encoding, rotate_180,
    mirror_horizontal,
    get_logger());

  std::string error;
  if (!left_worker_->start(error)) {
    throw std::runtime_error("left capture start failed: " + error);
  }
  if (!right_worker_->start(error)) {
    left_worker_->stop();
    throw std::runtime_error("right capture start failed: " + error);
  }

  publish_history_.clear();
  last_stats_log_sec_ = now_sec();
  timer_ = create_wall_timer(
    std::chrono::duration<double>(publish_period_sec_),
    std::bind(&Imx219StereoCaptureNode::on_timer, this));

  RCLCPP_INFO(
    get_logger(),
    "imx219_stereo_capture_node started: %dx%d@%d, output=%s, rotate_180=%s, mirror_horizontal=%s, swap_left_right=%s, target_publish_hz=%.1f",
    width, height, fps, output_encoding.c_str(), rotate_180 ? "true" : "false",
    mirror_horizontal ? "true" : "false", swap_left_right_ ? "true" : "false", target_publish_hz);
}

Imx219StereoCaptureNode::~Imx219StereoCaptureNode()
{
  if (left_worker_ != nullptr) {
    left_worker_->stop();
  }
  if (right_worker_ != nullptr) {
    right_worker_->stop();
  }
}

sensor_msgs::msg::CameraInfo Imx219StereoCaptureNode::load_camera_info(
  const std::string & calib_path,
  const std::string & side,
  const std::string & frame_id,
  int output_width,
  int output_height) const
{
  const YAML::Node root = YAML::LoadFile(calib_path);
  const auto k_key = side == "left" ? "K1" : "K2";
  const auto d_key = side == "left" ? "D1" : "D2";
  const auto p_key = side == "left" ? "P1" : "P2";

  const auto k = parse_flat_array<9>(root[k_key]);
  const auto d = parse_vector(root[d_key]);
  const auto p = parse_flat_array<12>(root[p_key]);
  const auto img_size = parse_flat_array<2>(root["img_size"]);
  const double calib_width = img_size[0];
  const double calib_height = img_size[1];
  const double scale_x = static_cast<double>(output_width) / calib_width;
  const double scale_y = static_cast<double>(output_height) / calib_height;

  sensor_msgs::msg::CameraInfo ci;
  ci.header.frame_id = frame_id;
  ci.width = static_cast<uint32_t>(output_width);
  ci.height = static_cast<uint32_t>(output_height);
  ci.distortion_model = "plumb_bob";
  ci.d = d;

  ci.k = {
    k[0] * scale_x, k[1], k[2] * scale_x,
    k[3], k[4] * scale_y, k[5] * scale_y,
    k[6], k[7], k[8]
  };
  ci.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  };
  ci.p = {
    p[0] * scale_x, p[1], p[2] * scale_x, p[3] * scale_x,
    p[4], p[5] * scale_y, p[6] * scale_y, p[7] * scale_y,
    p[8], p[9], p[10], p[11]
  };
  return ci;
}

sensor_msgs::msg::CameraInfo Imx219StereoCaptureNode::camera_info_with_stamp(
  const sensor_msgs::msg::CameraInfo & templ,
  double stamp_sec) const
{
  auto msg = templ;
  const int32_t sec = static_cast<int32_t>(stamp_sec);
  const uint32_t nanosec = static_cast<uint32_t>((stamp_sec - static_cast<double>(sec)) * 1e9);
  msg.header.stamp.sec = sec;
  msg.header.stamp.nanosec = nanosec;
  return msg;
}

sensor_msgs::msg::Image Imx219StereoCaptureNode::image_msg_from_frame(
  const FrameSnapshot & frame,
  const std::string & frame_id) const
{
  sensor_msgs::msg::Image msg;
  const int32_t sec = static_cast<int32_t>(frame.stamp_sec);
  const uint32_t nanosec = static_cast<uint32_t>(
    (frame.stamp_sec - static_cast<double>(sec)) * 1e9);
  msg.header.stamp.sec = sec;
  msg.header.stamp.nanosec = nanosec;
  msg.header.frame_id = frame_id;
  msg.height = static_cast<uint32_t>(frame.height);
  msg.width = static_cast<uint32_t>(frame.width);
  msg.encoding = frame.encoding;
  msg.is_bigendian = false;
  msg.step = static_cast<uint32_t>(frame.step);
  msg.data = frame.data;
  return msg;
}

void Imx219StereoCaptureNode::publish_pair_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  const double stamp = now_sec();
  array.header.stamp.sec = static_cast<int32_t>(stamp);
  array.header.stamp.nanosec = static_cast<uint32_t>((stamp - static_cast<double>(array.header.stamp.sec)) * 1e9);

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_name() + std::string(": stereo_pairing");
  status.hardware_id = "imx219_stereo_pair";
  status.level = (dropped_skew_count_ == 0U && dropped_stale_count_ == 0U) ?
    diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK ? "ok" : "pairing_drops_observed";

  const auto p95 = skew_history_ms_.empty() ? std::numeric_limits<double>::quiet_NaN() :
    percentile(std::vector<double>(skew_history_ms_.begin(), skew_history_ms_.end()), 0.95);
  const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(std::move(entry));
    };
  add("current_skew_ms", format_metric(last_skew_ms_, 3));
  add("window_p95_skew_ms", format_metric(p95, 3));
  add("left_seq", std::to_string(last_pair_left_seq_));
  add("right_seq", std::to_string(last_pair_right_seq_));
  add("published_pairs", std::to_string(publish_count_));
  add("drop_skew", std::to_string(dropped_skew_count_));
  add("drop_stale", std::to_string(dropped_stale_count_));
  add("wait_pair", std::to_string(waiting_for_pair_count_));
  add("left_failures", std::to_string(left_worker_->total_failures()));
  add("right_failures", std::to_string(right_worker_->total_failures()));
  array.status.push_back(std::move(status));
  pair_diagnostics_pub_->publish(array);
}

double Imx219StereoCaptureNode::publish_rate_hz() const
{
  if (publish_history_.size() < 2) {
    return 0.0;
  }
  const double elapsed = publish_history_.back() - publish_history_.front();
  if (elapsed <= 0.0) {
    return 0.0;
  }
  return static_cast<double>(publish_history_.size() - 1U) / elapsed;
}

std::string Imx219StereoCaptureNode::skew_summary() const
{
  if (skew_history_ms_.empty()) {
    return "min=nan median=nan p95=nan";
  }
  std::vector<double> values(skew_history_ms_.begin(), skew_history_ms_.end());
  const double min_value = *std::min_element(values.begin(), values.end());
  const double median_value = percentile(values, 0.5);
  const double p95_value = percentile(values, 0.95);
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(2);
  stream << "min=" << min_value << " median=" << median_value << " p95=" << p95_value;
  return stream.str();
}

void Imx219StereoCaptureNode::on_timer()
{
  const double current_sec = now_sec();
  const auto left_frames = left_worker_->recent_frames();
  const auto right_frames = right_worker_->recent_frames();
  if (left_frames.empty() || right_frames.empty()) {
    ++waiting_for_pair_count_;
    publish_pair_diagnostics();
    maybe_log_stats(current_sec);
    return;
  }

  bool saw_fresh_candidate = false;
  bool saw_new_candidate = false;
  double best_any_skew_sec = std::numeric_limits<double>::infinity();
  const FrameSnapshot * best_left = nullptr;
  const FrameSnapshot * best_right = nullptr;
  double best_pair_skew_sec = std::numeric_limits<double>::infinity();
  uint64_t best_pair_score = 0;

  for (auto left_it = left_frames.rbegin(); left_it != left_frames.rend(); ++left_it) {
    const bool left_is_new = reuse_latest_frame_ || left_it->seq > last_pair_left_seq_;
    if (!left_is_new) {
      continue;
    }
    for (auto right_it = right_frames.rbegin(); right_it != right_frames.rend(); ++right_it) {
      const bool right_is_new = reuse_latest_frame_ || right_it->seq > last_pair_right_seq_;
      if (!right_is_new) {
        continue;
      }
      saw_new_candidate = true;
      const bool stale_pair =
        (current_sec - left_it->stamp_sec) > frame_stale_sec_ ||
        (current_sec - right_it->stamp_sec) > frame_stale_sec_;
      if (stale_pair) {
        continue;
      }
      saw_fresh_candidate = true;
      const double skew_sec = std::fabs(left_it->stamp_sec - right_it->stamp_sec);
      best_any_skew_sec = std::min(best_any_skew_sec, skew_sec);
      if (skew_sec > pair_max_skew_sec_) {
        continue;
      }
      const uint64_t pair_score = left_it->seq + right_it->seq;
      if (
        best_left == nullptr || skew_sec < best_pair_skew_sec ||
        (std::fabs(skew_sec - best_pair_skew_sec) < 1e-9 && pair_score > best_pair_score))
      {
        best_left = &(*left_it);
        best_right = &(*right_it);
        best_pair_skew_sec = skew_sec;
        best_pair_score = pair_score;
      }
    }
  }

  if (best_left == nullptr || best_right == nullptr) {
    last_skew_ms_ = std::isfinite(best_any_skew_sec) ? (best_any_skew_sec * 1000.0) : 0.0;
    if (!saw_new_candidate) {
      ++waiting_for_pair_count_;
    } else if (!saw_fresh_candidate) {
      ++dropped_stale_count_;
    } else {
      ++dropped_skew_count_;
    }
    publish_pair_diagnostics();
    maybe_log_stats(current_sec);
    return;
  }

  last_skew_ms_ = best_pair_skew_sec * 1000.0;
  skew_history_ms_.push_back(last_skew_ms_);
  if (skew_history_ms_.size() > 120) {
    skew_history_ms_.pop_front();
  }

  const double publish_start = now_sec();
  const FrameSnapshot & left_output_frame = swap_left_right_ ? *best_right : *best_left;
  const FrameSnapshot & right_output_frame = swap_left_right_ ? *best_left : *best_right;
  // Do not conceal capture skew: each output retains its own FrameSnapshot stamp.
  left_image_pub_->publish(image_msg_from_frame(left_output_frame, left_frame_id_));
  right_image_pub_->publish(image_msg_from_frame(right_output_frame, right_frame_id_));
  if (publish_info_ && has_camera_info_) {
    left_info_pub_->publish(camera_info_with_stamp(left_info_template_, left_output_frame.stamp_sec));
    right_info_pub_->publish(camera_info_with_stamp(right_info_template_, right_output_frame.stamp_sec));
  }
  last_publish_duration_ms_ = (now_sec() - publish_start) * 1000.0;

  ++publish_count_;
  publish_history_.push_back(current_sec);
  if (publish_history_.size() > 120) {
    publish_history_.pop_front();
  }
  last_pair_left_seq_ = best_left->seq;
  last_pair_right_seq_ = best_right->seq;

  publish_pair_diagnostics();

  maybe_log_stats(current_sec);
}

void Imx219StereoCaptureNode::maybe_log_stats(double now_sec_value)
{
  if ((now_sec_value - last_stats_log_sec_) < log_stats_period_sec_) {
    return;
  }
  last_stats_log_sec_ = now_sec_value;
  const auto publish_delta = publish_count_ - last_logged_publish_count_;
  const auto skew_delta = dropped_skew_count_ - last_logged_dropped_skew_count_;
  const auto stale_delta = dropped_stale_count_ - last_logged_dropped_stale_count_;
  const auto wait_delta = waiting_for_pair_count_ - last_logged_waiting_for_pair_count_;
  last_logged_publish_count_ = publish_count_;
  last_logged_dropped_skew_count_ = dropped_skew_count_;
  last_logged_dropped_stale_count_ = dropped_stale_count_;
  last_logged_waiting_for_pair_count_ = waiting_for_pair_count_;
  RCLCPP_INFO(
    get_logger(),
    "stats publish_hz=%.2f left_capture_hz=%.2f right_capture_hz=%.2f published=%llu(+%llu) "
    "drop_skew=%llu(+%llu) drop_stale=%llu(+%llu) wait_pair=%llu(+%llu) "
    "last_skew_ms=%.2f skew_window={%s} publish_ms=%.2f "
    "last_pair_seq=%llu/%llu left_failures=%llu right_failures=%llu",
    publish_rate_hz(),
    left_worker_->capture_rate_hz(),
    right_worker_->capture_rate_hz(),
    static_cast<unsigned long long>(publish_count_),
    static_cast<unsigned long long>(publish_delta),
    static_cast<unsigned long long>(dropped_skew_count_),
    static_cast<unsigned long long>(skew_delta),
    static_cast<unsigned long long>(dropped_stale_count_),
    static_cast<unsigned long long>(stale_delta),
    static_cast<unsigned long long>(waiting_for_pair_count_),
    static_cast<unsigned long long>(wait_delta),
    last_skew_ms_,
    skew_summary().c_str(),
    last_publish_duration_ms_,
    static_cast<unsigned long long>(last_pair_left_seq_),
    static_cast<unsigned long long>(last_pair_right_seq_),
    static_cast<unsigned long long>(left_worker_->total_failures()),
    static_cast<unsigned long long>(right_worker_->total_failures()));
}

}  // namespace deyes_capture_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<deyes_capture_cpp::Imx219StereoCaptureNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
