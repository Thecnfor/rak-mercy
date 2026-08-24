#include "deyes_capture_cpp/cuda_stereo_depth_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

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

cv::Mat row_vector_from_values(
  const std::vector<double> & values,
  const std::string & name,
  int min_size)
{
  if (static_cast<int>(values.size()) < min_size) {
    throw std::runtime_error(name + " must contain at least " + std::to_string(min_size) + " values");
  }
  cv::Mat mat(1, static_cast<int>(values.size()), CV_64F);
  std::memcpy(mat.data, values.data(), sizeof(double) * values.size());
  return mat;
}

cv::Mat col_vector_from_values(
  const std::vector<double> & values,
  const std::string & name,
  int expected_size)
{
  if (static_cast<int>(values.size()) != expected_size) {
    throw std::runtime_error(
      name + " must contain exactly " + std::to_string(expected_size) + " values");
  }
  cv::Mat mat(expected_size, 1, CV_64F);
  std::memcpy(mat.data, values.data(), sizeof(double) * values.size());
  return mat;
}

template<int Rows, int Cols>
cv::Mat matrix_from_node(const YAML::Node & node)
{
  const auto values = parse_flat_array<Rows * Cols>(node);
  cv::Mat mat(Rows, Cols, CV_64F);
  std::memcpy(mat.data, values.data(), sizeof(double) * values.size());
  return mat;
}

int64_t stamp_to_ns(const rclcpp::Time & stamp)
{
  return static_cast<int64_t>(stamp.nanoseconds());
}

double percentile_value(std::vector<double> values, double percentile)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  percentile = std::clamp(percentile, 0.0, 100.0);
  const double rank = (percentile / 100.0) * static_cast<double>(values.size() - 1U);
  const auto lower_index = static_cast<std::size_t>(std::floor(rank));
  const auto upper_index = static_cast<std::size_t>(std::ceil(rank));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(lower_index), values.end());
  const double lower_value = values[lower_index];
  if (upper_index == lower_index) {
    return lower_value;
  }
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(upper_index), values.end());
  const double upper_value = values[upper_index];
  const double alpha = rank - static_cast<double>(lower_index);
  return lower_value + (upper_value - lower_value) * alpha;
}

std::string compact_metric(double value, int precision = 3)
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

constexpr double kCenterRoiRatio = 0.60;

}  // namespace

namespace deyes_capture_cpp
{

CudaStereoDepthNode::CudaStereoDepthNode(const rclcpp::NodeOptions & options)
: Node("cuda_stereo_depth_node", options)
{
  declare_parameter<std::string>("calib_path", "");
  declare_parameter<std::string>("left_image_topic", "/x1/left_camera/image_raw");
  declare_parameter<std::string>("right_image_topic", "/x1/right_camera/image_raw");
  declare_parameter<double>("max_sync_diff_ms", 10.0);
  declare_parameter<double>("publish_period_sec", 1.0 / 30.0);
  declare_parameter<double>("frame_stale_sec", 0.2);
  declare_parameter<double>("min_depth_m", 0.20);
  declare_parameter<double>("max_depth_m", 1.50);
  declare_parameter<double>("processing_overrun_factor", 1.0);
  declare_parameter<double>("log_stats_period_sec", 2.0);
  declare_parameter<bool>("enable_wls_filter", true);
  declare_parameter<double>("wls_lambda", 8000.0);
  declare_parameter<double>("wls_sigma_color", 2.0);
  declare_parameter<int>("frame_queue_size", 8);
  declare_parameter<int>("min_disparity", 0);
  declare_parameter<int>("num_disparities", 64);
  declare_parameter<int>("block_size", 19);
  declare_parameter<int>("prefilter_type", 1);
  declare_parameter<int>("texture_threshold", 0);
  declare_parameter<int>("uniqueness_ratio", 0);
  declare_parameter<int>("speckle_window_size", 0);
  declare_parameter<int>("speckle_range", 0);
  declare_parameter<int>("disp12_max_diff", 0);
  declare_parameter<int>("median_ksize", 9);
  declare_parameter<bool>("publish_debug_rect", true);
  declare_parameter<bool>("publish_debug_mask", true);
  declare_parameter<std::string>("disparity_topic", "/x1/stereo/disparity");
  declare_parameter<std::string>("depth_topic", "/x1/stereo/depth");
  declare_parameter<std::string>("left_rect_camera_info_topic", "/x1/stereo/left/camera_info_rect");
  declare_parameter<std::string>("debug_left_rect_topic", "/x1/stereo/debug/left_rect");
  declare_parameter<std::string>("debug_right_rect_topic", "/x1/stereo/debug/right_rect");
  declare_parameter<std::string>("debug_valid_mask_topic", "/x1/stereo/debug/valid_mask");
  declare_parameter<std::string>("state_topic", "~/status");
  declare_parameter<std::string>("state_detail_topic", "~/status_detail");

  const auto calib_path = get_parameter("calib_path").as_string();
  if (calib_path.empty()) {
    throw std::runtime_error("cuda_stereo_depth_node requires calib_path");
  }
  calibration_ = load_stereo_calibration(calib_path);

  max_sync_diff_ns_ = static_cast<int64_t>(get_parameter("max_sync_diff_ms").as_double() * 1e6);
  publish_period_ns_ = static_cast<int64_t>(get_parameter("publish_period_sec").as_double() * 1e9);
  frame_stale_ns_ = static_cast<int64_t>(get_parameter("frame_stale_sec").as_double() * 1e9);
  min_depth_m_ = get_parameter("min_depth_m").as_double();
  max_depth_m_ = get_parameter("max_depth_m").as_double();
  processing_overrun_factor_ = get_parameter("processing_overrun_factor").as_double();
  log_stats_period_sec_ = get_parameter("log_stats_period_sec").as_double();
  enable_wls_filter_ = get_parameter("enable_wls_filter").as_bool();
  wls_lambda_ = get_parameter("wls_lambda").as_double();
  wls_sigma_color_ = get_parameter("wls_sigma_color").as_double();
  frame_queue_size_ = static_cast<std::size_t>(get_parameter("frame_queue_size").as_int());
  min_disparity_ = get_parameter("min_disparity").as_int();
  texture_threshold_ = get_parameter("texture_threshold").as_int();
  uniqueness_ratio_ = get_parameter("uniqueness_ratio").as_int();
  speckle_window_size_ = get_parameter("speckle_window_size").as_int();
  speckle_range_ = get_parameter("speckle_range").as_int();
  disp12_max_diff_ = get_parameter("disp12_max_diff").as_int();
  publish_debug_rect_ = get_parameter("publish_debug_rect").as_bool();
  publish_debug_mask_ = get_parameter("publish_debug_mask").as_bool();

  const int num_disparities = get_parameter("num_disparities").as_int();
  const int block_size = get_parameter("block_size").as_int();
  const int median_ksize = get_parameter("median_ksize").as_int();
  if (num_disparities <= 0 || (num_disparities % 16) != 0) {
    throw std::runtime_error("num_disparities must be positive and divisible by 16");
  }
  if (block_size <= 0 || (block_size % 2) == 0) {
    throw std::runtime_error("block_size must be a positive odd integer");
  }
  if (median_ksize <= 1 || (median_ksize % 2) == 0) {
    throw std::runtime_error("median_ksize must be an odd integer greater than one");
  }
  if (publish_period_ns_ <= 0) {
    throw std::runtime_error("publish_period_sec must be positive");
  }
  if (frame_stale_ns_ <= 0) {
    throw std::runtime_error("frame_stale_sec must be positive");
  }
  if (frame_queue_size_ == 0U) {
    throw std::runtime_error("frame_queue_size must be positive");
  }
  if (processing_overrun_factor_ <= 0.0) {
    throw std::runtime_error("processing_overrun_factor must be positive");
  }
  if (wls_lambda_ < 0.0) {
    throw std::runtime_error("wls_lambda must be non-negative");
  }
  if (wls_sigma_color_ <= 0.0) {
    throw std::runtime_error("wls_sigma_color must be positive");
  }
  if (texture_threshold_ < 0 || uniqueness_ratio_ < 0 || speckle_window_size_ < 0 ||
    speckle_range_ < 0 || disp12_max_diff_ < 0)
  {
    throw std::runtime_error("StereoBM quality parameters must be non-negative");
  }

  median_filter_ = cv::cuda::createMedianFilter(CV_8UC1, median_ksize);
  stereo_bm_ = cv::cuda::createStereoBM(num_disparities, block_size);
  stereo_bm_->setMinDisparity(min_disparity_);
  stereo_bm_->setPreFilterType(get_parameter("prefilter_type").as_int());
  stereo_bm_->setTextureThreshold(texture_threshold_);
  stereo_bm_->setUniquenessRatio(uniqueness_ratio_);
  stereo_bm_->setSpeckleWindowSize(speckle_window_size_);
  stereo_bm_->setSpeckleRange(speckle_range_);
  stereo_bm_->setDisp12MaxDiff(disp12_max_diff_);
  if (enable_wls_filter_) {
    restart_wls_filter();
  }

  const auto qos = rclcpp::SensorDataQoS();
  disparity_pub_ = create_publisher<sensor_msgs::msg::Image>(
    get_parameter("disparity_topic").as_string(), qos);
  depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
    get_parameter("depth_topic").as_string(), qos);
  left_rect_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    get_parameter("left_rect_camera_info_topic").as_string(), qos);
  if (publish_debug_rect_) {
    left_rect_pub_ = create_publisher<sensor_msgs::msg::Image>(
      get_parameter("debug_left_rect_topic").as_string(), qos);
    right_rect_pub_ = create_publisher<sensor_msgs::msg::Image>(
      get_parameter("debug_right_rect_topic").as_string(), qos);
  }
  if (publish_debug_mask_) {
    valid_mask_pub_ = create_publisher<sensor_msgs::msg::Image>(
      get_parameter("debug_valid_mask_topic").as_string(), qos);
  }
  state_pub_ = create_publisher<std_msgs::msg::String>(
    get_parameter("state_topic").as_string(), 10);
  state_detail_pub_ = create_publisher<std_msgs::msg::String>(
    get_parameter("state_detail_topic").as_string(), 10);

  left_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    get_parameter("left_image_topic").as_string(),
    qos,
    std::bind(&CudaStereoDepthNode::on_left_image, this, std::placeholders::_1));
  right_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    get_parameter("right_image_topic").as_string(),
    qos,
    std::bind(&CudaStereoDepthNode::on_right_image, this, std::placeholders::_1));
  timer_ = create_wall_timer(
    std::chrono::duration<double>(get_parameter("publish_period_sec").as_double()),
    std::bind(&CudaStereoDepthNode::on_timer, this));

  RCLCPP_INFO(
    get_logger(),
    "cuda_stereo_depth_node started: disparity=%s depth=%s rectified_info=%s calib=%s validated=%s queue=%zu "
    "max_sync_diff_ms=%.2f frame_stale_sec=%.3f wls=%s texture_threshold=%d "
    "uniqueness_ratio=%d speckle_window_size=%d speckle_range=%d disp12_max_diff=%d",
    get_parameter("disparity_topic").as_string().c_str(),
    get_parameter("depth_topic").as_string().c_str(),
    get_parameter("left_rect_camera_info_topic").as_string().c_str(),
    calib_path.c_str(),
    calibration_.validated ? "true" : "false",
    frame_queue_size_,
    static_cast<double>(max_sync_diff_ns_) / 1e6,
    static_cast<double>(frame_stale_ns_) / 1e9,
    enable_wls_filter_ ? "on" : "off",
    texture_threshold_,
    uniqueness_ratio_,
    speckle_window_size_,
    speckle_range_,
    disp12_max_diff_);
}

StereoCalibration CudaStereoDepthNode::load_stereo_calibration(const std::string & calib_path) const
{
  const YAML::Node root = YAML::LoadFile(calib_path);
  const std::array<const char *, 10> metadata_keys = {
    "calibration_id", "robot_id", "camera_pair_id", "img_size", "board_inner_corners",
    "square_size_m", "reproj_rms_px", "epipolar_p95_px", "date", "source"};
  for (const auto * key : metadata_keys) {
    if (!root[key]) {
      throw std::runtime_error("stereo calibration missing required metadata key: " + std::string(key));
    }
  }
  if (!root["validated"]) {
    throw std::runtime_error("stereo calibration missing required metadata key: validated");
  }
  const auto board_inner_corners = parse_flat_array<2>(root["board_inner_corners"]);
  for (const double value : board_inner_corners) {
    if (!std::isfinite(value) || value < 4.0 || std::floor(value) != value) {
      throw std::runtime_error(
              "stereo calibration board_inner_corners must contain explicit integer dimensions at least 4x4");
    }
  }
  const auto img_size = parse_flat_array<2>(root["img_size"]);
  for (const double value : img_size) {
    if (!std::isfinite(value) || value < 1.0 || std::floor(value) != value) {
      throw std::runtime_error("stereo calibration img_size must contain positive integer dimensions");
    }
  }
  const auto d1_values = parse_vector(root["D1"]);
  const auto d2_values = parse_vector(root["D2"]);
  const auto t_values = parse_vector(root["T"]);

  StereoCalibration calibration;
  calibration.image_size = cv::Size(static_cast<int>(img_size[0]), static_cast<int>(img_size[1]));
  calibration.k1 = matrix_from_node<3, 3>(root["K1"]);
  calibration.d1 = row_vector_from_values(d1_values, "D1", 4);
  calibration.k2 = matrix_from_node<3, 3>(root["K2"]);
  calibration.d2 = row_vector_from_values(d2_values, "D2", 4);
  calibration.r = matrix_from_node<3, 3>(root["R"]);
  calibration.t = col_vector_from_values(t_values, "T", 3);
  calibration.baseline_m = root["baseline_m"] ? std::abs(root["baseline_m"].as<double>()) :
    std::abs(calibration.t.at<double>(0, 0));
  calibration.fx = root["fx"] ? root["fx"].as<double>() : calibration.k1.at<double>(0, 0);
  calibration.calibration_id = root["calibration_id"].as<std::string>();
  calibration.robot_id = root["robot_id"].as<std::string>();
  calibration.camera_pair_id = root["camera_pair_id"].as<std::string>();
  calibration.source = root["source"].as<std::string>();
  calibration.validated = root["validated"].as<bool>();
  if (calibration.validated) {
    if (calibration.source != "physical_checkerboard" && calibration.source != "physical_charuco") {
      throw std::runtime_error("validated stereo calibration must use source=physical_checkerboard or physical_charuco");
    }
    if (root["square_size_m"].IsNull() || root["reproj_rms_px"].IsNull() || root["epipolar_p95_px"].IsNull()) {
      throw std::runtime_error("validated stereo calibration requires measured board and error metadata");
    }
    if (calibration.calibration_id.empty() || calibration.robot_id.empty() || calibration.camera_pair_id.empty()) {
      throw std::runtime_error("validated stereo calibration requires non-empty identity metadata");
    }
    const double square_size_m = root["square_size_m"].as<double>();
    const double reproj_rms_px = root["reproj_rms_px"].as<double>();
    const double epipolar_p95_px = root["epipolar_p95_px"].as<double>();
    if (!std::isfinite(square_size_m) || square_size_m <= 0.0 ||
      !std::isfinite(reproj_rms_px) || reproj_rms_px > 0.50 ||
      !std::isfinite(epipolar_p95_px) || epipolar_p95_px > 0.50)
    {
      throw std::runtime_error("validated stereo calibration measurement gates are not satisfied");
    }
    if (calibration.image_size.width != 640 || calibration.image_size.height != 360) {
      throw std::runtime_error("validated stereo calibration resolution must be 640x360");
    }
  }
  return calibration;
}

cv::Mat CudaStereoDepthNode::image_msg_to_mat(const sensor_msgs::msg::Image & msg) const
{
  int type = 0;
  int channels = 0;
  if (msg.encoding == "mono8" || msg.encoding == "8UC1") {
    type = CV_8UC1;
    channels = 1;
  } else if (msg.encoding == "bgr8" || msg.encoding == "rgb8") {
    type = CV_8UC3;
    channels = 3;
  } else {
    throw std::runtime_error("Unsupported image encoding: " + msg.encoding);
  }

  cv::Mat result(msg.height, msg.width, type);
  const std::size_t row_bytes = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(channels);
  const auto * src = msg.data.data();
  for (std::size_t row = 0; row < msg.height; ++row) {
    std::memcpy(result.ptr(static_cast<int>(row)), src + row * msg.step, row_bytes);
  }
  return result;
}

sensor_msgs::msg::Image CudaStereoDepthNode::make_image_msg(
  const cv::Mat & frame,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::string & encoding) const
{
  const cv::Mat contiguous = frame.isContinuous() ? frame : frame.clone();

  sensor_msgs::msg::Image msg;
  msg.header.stamp = builtin_interfaces::msg::Time();
  msg.header.stamp.sec = static_cast<int32_t>(stamp.seconds());
  msg.header.stamp.nanosec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);
  msg.header.frame_id = frame_id;
  msg.height = static_cast<uint32_t>(contiguous.rows);
  msg.width = static_cast<uint32_t>(contiguous.cols);
  msg.encoding = encoding;
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(contiguous.step);
  msg.data.resize(static_cast<std::size_t>(contiguous.rows) * contiguous.step);
  std::memcpy(msg.data.data(), contiguous.data, msg.data.size());
  return msg;
}

void CudaStereoDepthNode::restart_wls_filter()
{
  wls_filter_ = cv::ximgproc::createDisparityWLSFilter(stereo_bm_);
  wls_filter_->setLambda(wls_lambda_);
  wls_filter_->setSigmaColor(wls_sigma_color_);
}

void CudaStereoDepthNode::enqueue_frame(std::deque<FrameBundle> & queue, FrameBundle && bundle)
{
  queue.push_back(std::move(bundle));
  while (queue.size() > frame_queue_size_) {
    queue.pop_front();
  }
}

void CudaStereoDepthNode::prune_queues_locked(int64_t now_ns)
{
  const auto prune_queue = [this, now_ns](std::deque<FrameBundle> & queue, uint64_t processed_seq) {
      while (!queue.empty()) {
        const FrameBundle & frame = queue.front();
        const bool already_processed = frame.seq <= processed_seq;
        const bool stale = (now_ns - stamp_to_ns(frame.stamp)) > frame_stale_ns_;
        if (!already_processed && !stale) {
          break;
        }
        queue.pop_front();
      }
    };

  prune_queue(left_frames_, last_processed_left_seq_);
  prune_queue(right_frames_, last_processed_right_seq_);
}

CudaStereoDepthNode::PairSelection CudaStereoDepthNode::select_best_pair(
  const std::deque<FrameBundle> & left_frames,
  const std::deque<FrameBundle> & right_frames,
  int64_t now_ns) const
{
  PairSelection result;
  if (left_frames.empty() || right_frames.empty()) {
    std::vector<std::string> missing_parts;
    if (left_frames.empty()) {
      missing_parts.emplace_back("left_image");
    }
    if (right_frames.empty()) {
      missing_parts.emplace_back("right_image");
    }
    std::ostringstream stream;
    for (std::size_t index = 0; index < missing_parts.size(); ++index) {
      if (index != 0U) {
        stream << ",";
      }
      stream << missing_parts[index];
    }
    result.detail = stream.str().empty() ? "missing_pair_candidate" : stream.str();
    return result;
  }

  bool saw_new_candidate = false;
  bool saw_fresh_candidate = false;
  int64_t best_in_window_diff_ns = std::numeric_limits<int64_t>::max();
  uint64_t best_pair_score = 0;
  uint64_t best_left_seq = 0;
  uint64_t best_right_seq = 0;
  int64_t best_any_fresh_diff_ns = std::numeric_limits<int64_t>::max();

  for (auto left_it = left_frames.rbegin(); left_it != left_frames.rend(); ++left_it) {
    if (left_it->seq <= last_processed_left_seq_) {
      continue;
    }
    for (auto right_it = right_frames.rbegin(); right_it != right_frames.rend(); ++right_it) {
      if (right_it->seq <= last_processed_right_seq_) {
        continue;
      }
      saw_new_candidate = true;
      const bool stale_pair =
        (now_ns - stamp_to_ns(left_it->stamp)) > frame_stale_ns_ ||
        (now_ns - stamp_to_ns(right_it->stamp)) > frame_stale_ns_;
      if (stale_pair) {
        continue;
      }
      saw_fresh_candidate = true;
      const int64_t diff_ns = std::llabs(stamp_to_ns(left_it->stamp) - stamp_to_ns(right_it->stamp));
      best_any_fresh_diff_ns = std::min(best_any_fresh_diff_ns, diff_ns);
      if (diff_ns > max_sync_diff_ns_) {
        continue;
      }
      const uint64_t pair_score = left_it->seq + right_it->seq;
      if (
        !result.left.has_value() || diff_ns < best_in_window_diff_ns ||
        (diff_ns == best_in_window_diff_ns && pair_score > best_pair_score))
      {
        result.left = *left_it;
        result.right = *right_it;
        best_in_window_diff_ns = diff_ns;
        best_pair_score = pair_score;
        best_left_seq = left_it->seq;
        best_right_seq = right_it->seq;
      }
    }
  }

  if (result.left.has_value() && result.right.has_value()) {
    result.state = DepthStreamState::kOk;
    result.best_diff_ns = best_in_window_diff_ns;
    result.detail =
      "selected left_seq=" + std::to_string(best_left_seq) +
      " right_seq=" + std::to_string(best_right_seq);
    return result;
  }

  result.best_diff_ns = std::numeric_limits<int64_t>::max();
  if (!saw_new_candidate) {
    result.state = DepthStreamState::kMissingInput;
    result.detail = "waiting_for_new_frames";
  } else if (!saw_fresh_candidate) {
    result.state = DepthStreamState::kStaleFrame;
    result.detail = "all_unprocessed_pairs_are_stale";
  } else {
    result.state = DepthStreamState::kPairOutOfWindow;
    result.best_diff_ns = best_any_fresh_diff_ns;
    std::ostringstream stream;
    stream << "best_diff_ms=";
    stream.setf(std::ios::fixed);
    stream.precision(2);
    stream << (static_cast<double>(best_any_fresh_diff_ns) / 1e6);
    result.detail = stream.str();
  }
  return result;
}

void CudaStereoDepthNode::mark_pair_processed(uint64_t left_seq, uint64_t right_seq, int64_t now_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_processed_left_seq_ = std::max(last_processed_left_seq_, left_seq);
  last_processed_right_seq_ = std::max(last_processed_right_seq_, right_seq);
  prune_queues_locked(now_ns);
}

void CudaStereoDepthNode::publish_state(DepthStreamState state, const std::string & detail)
{
  std_msgs::msg::String status_msg;
  status_msg.data = state_to_string(state);
  state_pub_->publish(status_msg);

  const std::string detail_with_metrics = append_quality_metrics(detail);
  std_msgs::msg::String detail_msg;
  detail_msg.data = detail_with_metrics;
  state_detail_pub_->publish(detail_msg);

  if (!state_initialized_ || state != current_state_ || detail_with_metrics != current_state_detail_) {
    RCLCPP_INFO(
      get_logger(),
      "depth_stream_state=%s detail=%s",
      status_msg.data.c_str(),
      detail_with_metrics.c_str());
    current_state_ = state;
    current_state_detail_ = detail_with_metrics;
    state_initialized_ = true;
  }
}

void CudaStereoDepthNode::maybe_log_stats(int64_t now_ns)
{
  const double now_sec_value = static_cast<double>(now_ns) / 1e9;
  if ((now_sec_value - last_stats_log_sec_) < log_stats_period_sec_) {
    return;
  }
  last_stats_log_sec_ = now_sec_value;
  const auto published_delta = published_pairs_ - last_logged_published_pairs_;
  const auto missing_delta = missing_input_count_ - last_logged_missing_input_count_;
  const auto out_of_window_delta =
    pair_out_of_window_count_ - last_logged_pair_out_of_window_count_;
  const auto stale_delta = stale_frame_count_ - last_logged_stale_frame_count_;
  const auto overrun_delta =
    processing_overrun_count_ - last_logged_processing_overrun_count_;
  last_logged_published_pairs_ = published_pairs_;
  last_logged_missing_input_count_ = missing_input_count_;
  last_logged_pair_out_of_window_count_ = pair_out_of_window_count_;
  last_logged_stale_frame_count_ = stale_frame_count_;
  last_logged_processing_overrun_count_ = processing_overrun_count_;

  double min_pair_diff_ms = std::numeric_limits<double>::quiet_NaN();
  double max_pair_diff_ms = std::numeric_limits<double>::quiet_NaN();
  if (!pair_diff_history_ms_.empty()) {
    min_pair_diff_ms = *std::min_element(pair_diff_history_ms_.begin(), pair_diff_history_ms_.end());
    max_pair_diff_ms = *std::max_element(pair_diff_history_ms_.begin(), pair_diff_history_ms_.end());
  }

  std::size_t left_queue_size = 0U;
  std::size_t right_queue_size = 0U;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    left_queue_size = left_frames_.size();
    right_queue_size = right_frames_.size();
  }

  RCLCPP_INFO(
    get_logger(),
    "depth_stats published=%llu(+%llu) missing_input=%llu(+%llu) pair_out_of_window=%llu(+%llu) "
    "stale_frame=%llu(+%llu) processing_overrun=%llu(+%llu) last_pair_diff_ms=%.2f "
    "pair_diff_window_ms={min=%.2f max=%.2f} last_processing_ms=%.2f "
    "valid_ratio_1m=%.3f valid_pixels_1m=%llu coverage_ratio_center_roi=%.3f "
    "median_depth_1m=%s p95_processing_ms=%s queue=%zu/%zu state=%s",
    static_cast<unsigned long long>(published_pairs_),
    static_cast<unsigned long long>(published_delta),
    static_cast<unsigned long long>(missing_input_count_),
    static_cast<unsigned long long>(missing_delta),
    static_cast<unsigned long long>(pair_out_of_window_count_),
    static_cast<unsigned long long>(out_of_window_delta),
    static_cast<unsigned long long>(stale_frame_count_),
    static_cast<unsigned long long>(stale_delta),
    static_cast<unsigned long long>(processing_overrun_count_),
    static_cast<unsigned long long>(overrun_delta),
    last_pair_diff_ms_,
    min_pair_diff_ms,
    max_pair_diff_ms,
    last_processing_ms_,
    last_quality_metrics_.valid_ratio_1m,
    static_cast<unsigned long long>(last_quality_metrics_.valid_pixels_1m),
    last_quality_metrics_.coverage_ratio_center_roi,
    compact_metric(last_quality_metrics_.median_depth_1m, 3).c_str(),
    compact_metric(last_quality_metrics_.p95_processing_ms, 2).c_str(),
    left_queue_size,
    right_queue_size,
    state_to_string(current_state_));
}

const char * CudaStereoDepthNode::state_to_string(DepthStreamState state)
{
  switch (state) {
    case DepthStreamState::kOk:
      return "ok";
    case DepthStreamState::kMissingInput:
      return "missing_input";
    case DepthStreamState::kPairOutOfWindow:
      return "pair_out_of_window";
    case DepthStreamState::kStaleFrame:
      return "stale_frame";
    case DepthStreamState::kProcessingOverrun:
      return "processing_overrun";
    default:
      return "unknown";
  }
}

void CudaStereoDepthNode::ensure_rectify_maps(int width, int height)
{
  if (rectify_size_ == cv::Size(width, height)) {
    return;
  }

  // Debug calibration may be rendered at another resolution.  Scale the input
  // intrinsics before stereoRectify; never resize maps made from 1280x720 K.
  const double scale_x = static_cast<double>(width) / static_cast<double>(calibration_.image_size.width);
  const double scale_y = static_cast<double>(height) / static_cast<double>(calibration_.image_size.height);
  cv::Mat k1 = calibration_.k1.clone();
  cv::Mat k2 = calibration_.k2.clone();
  for (cv::Mat * k : {&k1, &k2}) {
    k->at<double>(0, 0) *= scale_x;
    k->at<double>(0, 1) *= scale_x;
    k->at<double>(0, 2) *= scale_x;
    k->at<double>(1, 1) *= scale_y;
    k->at<double>(1, 2) *= scale_y;
  }

  cv::Mat r1;
  cv::Mat r2;
  cv::Mat p1;
  cv::Mat p2;
  cv::Mat q;
  cv::stereoRectify(
    k1,
    calibration_.d1,
    k2,
    calibration_.d2,
    cv::Size(width, height),
    calibration_.r,
    calibration_.t,
    r1,
    r2,
    p1,
    p2,
    q,
    cv::CALIB_ZERO_DISPARITY,
    0.0);

  calibration_.fx = p1.at<double>(0, 0);
  calibration_.baseline_m = std::abs(calibration_.t.at<double>(0, 0));
  rectified_p1_ = p1.clone();

  cv::Mat left_map1_cpu;
  cv::Mat left_map2_cpu;
  cv::Mat right_map1_cpu;
  cv::Mat right_map2_cpu;
  cv::initUndistortRectifyMap(
    k1, calibration_.d1, r1, p1, cv::Size(width, height), CV_32FC1,
    left_map1_cpu, left_map2_cpu);
  cv::initUndistortRectifyMap(
    k2, calibration_.d2, r2, p2, cv::Size(width, height), CV_32FC1,
    right_map1_cpu, right_map2_cpu);

  left_map1_gpu_.upload(left_map1_cpu);
  left_map2_gpu_.upload(left_map2_cpu);
  right_map1_gpu_.upload(right_map1_cpu);
  right_map2_gpu_.upload(right_map2_cpu);
  rectify_size_ = cv::Size(width, height);
}

sensor_msgs::msg::CameraInfo CudaStereoDepthNode::make_rectified_left_camera_info(
  const rclcpp::Time & stamp,
  const std::string & frame_id) const
{
  if (rectified_p1_.empty() || rectify_size_.width <= 0 || rectify_size_.height <= 0) {
    throw std::runtime_error("rectified CameraInfo requested before rectify maps are initialized");
  }
  sensor_msgs::msg::CameraInfo info;
  info.header.stamp.sec = static_cast<int32_t>(stamp.seconds());
  info.header.stamp.nanosec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);
  info.header.frame_id = frame_id;
  info.width = static_cast<uint32_t>(rectify_size_.width);
  info.height = static_cast<uint32_t>(rectify_size_.height);
  info.distortion_model = "plumb_bob";
  info.d.assign(5U, 0.0);
  info.k = {
    rectified_p1_.at<double>(0, 0), rectified_p1_.at<double>(0, 1), rectified_p1_.at<double>(0, 2),
    rectified_p1_.at<double>(1, 0), rectified_p1_.at<double>(1, 1), rectified_p1_.at<double>(1, 2),
    rectified_p1_.at<double>(2, 0), rectified_p1_.at<double>(2, 1), rectified_p1_.at<double>(2, 2)};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {
    rectified_p1_.at<double>(0, 0), rectified_p1_.at<double>(0, 1), rectified_p1_.at<double>(0, 2), rectified_p1_.at<double>(0, 3),
    rectified_p1_.at<double>(1, 0), rectified_p1_.at<double>(1, 1), rectified_p1_.at<double>(1, 2), rectified_p1_.at<double>(1, 3),
    rectified_p1_.at<double>(2, 0), rectified_p1_.at<double>(2, 1), rectified_p1_.at<double>(2, 2), rectified_p1_.at<double>(2, 3)};
  return info;
}

void CudaStereoDepthNode::on_left_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  FrameBundle bundle;
  bundle.stamp = rclcpp::Time(msg->header.stamp);
  bundle.frame_id = msg->header.frame_id;
  bundle.encoding = msg->encoding;
  bundle.width = static_cast<int>(msg->width);
  bundle.height = static_cast<int>(msg->height);
  bundle.image = image_msg_to_mat(*msg);

  std::lock_guard<std::mutex> lock(mutex_);
  bundle.seq = next_left_seq_++;
  enqueue_frame(left_frames_, std::move(bundle));
  prune_queues_locked(now().nanoseconds());
}

void CudaStereoDepthNode::on_right_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  FrameBundle bundle;
  bundle.stamp = rclcpp::Time(msg->header.stamp);
  bundle.frame_id = msg->header.frame_id;
  bundle.encoding = msg->encoding;
  bundle.width = static_cast<int>(msg->width);
  bundle.height = static_cast<int>(msg->height);
  bundle.image = image_msg_to_mat(*msg);

  std::lock_guard<std::mutex> lock(mutex_);
  bundle.seq = next_right_seq_++;
  enqueue_frame(right_frames_, std::move(bundle));
  prune_queues_locked(now().nanoseconds());
}

cv::Mat CudaStereoDepthNode::valid_mask_from_depth(const cv::Mat & depth) const
{
  cv::Mat mask(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0));
  for (int row = 0; row < depth.rows; ++row) {
    const auto * depth_ptr = depth.ptr<float>(row);
    auto * mask_ptr = mask.ptr<uint8_t>(row);
    for (int col = 0; col < depth.cols; ++col) {
      const float value = depth_ptr[col];
      mask_ptr[col] = std::isfinite(value) ? 255 : 0;
    }
  }
  return mask;
}

cv::Mat CudaStereoDepthNode::disparity_to_float32(const cv::Mat & disparity) const
{
  if (disparity.empty()) {
    return cv::Mat();
  }

  cv::Mat disparity_float;
  switch (disparity.type()) {
    case CV_8UC1:
      disparity.convertTo(disparity_float, CV_32F);
      break;
    case CV_16SC1:
    case CV_16UC1:
      disparity.convertTo(disparity_float, CV_32F, 1.0 / 16.0);
      break;
    case CV_32FC1:
      disparity_float = disparity.clone();
      break;
    default:
      throw std::runtime_error("Unsupported disparity image type for conversion");
  }
  return disparity_float;
}

cv::Mat CudaStereoDepthNode::compute_filtered_disparity(
  const cv::cuda::GpuMat & left_filtered_gpu,
  const cv::cuda::GpuMat & right_filtered_gpu,
  const cv::cuda::GpuMat & disparity_left_gpu,
  const cv::Mat & left_rect_cpu)
{
  cv::Mat disparity_left_raw;
  disparity_left_gpu.download(disparity_left_raw);
  if (!enable_wls_filter_ || !wls_filter_) {
    return disparity_to_float32(disparity_left_raw);
  }

  const int saved_min_disparity = stereo_bm_->getMinDisparity();
  const int saved_disp12_max_diff = stereo_bm_->getDisp12MaxDiff();
  const int saved_speckle_window_size = stereo_bm_->getSpeckleWindowSize();

  cv::Mat left_filtered_cpu;
  cv::Mat right_filtered_cpu;
  left_filtered_gpu.download(left_filtered_cpu);
  right_filtered_gpu.download(right_filtered_cpu);
  cv::Mat flipped_left_cpu;
  cv::Mat flipped_right_cpu;
  cv::flip(left_filtered_cpu, flipped_left_cpu, 1);
  cv::flip(right_filtered_cpu, flipped_right_cpu, 1);

  cv::cuda::GpuMat flipped_left_gpu;
  cv::cuda::GpuMat flipped_right_gpu;
  cv::cuda::GpuMat disparity_right_gpu;
  flipped_left_gpu.upload(flipped_left_cpu);
  flipped_right_gpu.upload(flipped_right_cpu);

  stereo_bm_->setMinDisparity(-(saved_min_disparity + stereo_bm_->getNumDisparities()) + 1);
  stereo_bm_->setDisp12MaxDiff(1000000);
  stereo_bm_->setSpeckleWindowSize(0);
  stereo_bm_->compute(flipped_right_gpu, flipped_left_gpu, disparity_right_gpu);
  stereo_bm_->setMinDisparity(saved_min_disparity);
  stereo_bm_->setDisp12MaxDiff(saved_disp12_max_diff);
  stereo_bm_->setSpeckleWindowSize(saved_speckle_window_size);

  cv::Mat disparity_right_raw;
  disparity_right_gpu.download(disparity_right_raw);

  cv::Mat disparity_filtered_raw;
  wls_filter_->filter(disparity_left_raw, left_rect_cpu, disparity_filtered_raw, disparity_right_raw);
  return disparity_to_float32(disparity_filtered_raw);
}

DepthQualityMetrics CudaStereoDepthNode::compute_depth_quality_metrics(
  const cv::Mat & depth,
  double processing_ms)
{
  DepthQualityMetrics metrics;
  if (depth.empty()) {
    return metrics;
  }

  processing_history_ms_.push_back(processing_ms);
  if (processing_history_ms_.size() > 120U) {
    processing_history_ms_.pop_front();
  }
  metrics.p95_processing_ms = percentile_value(
    std::vector<double>(processing_history_ms_.begin(), processing_history_ms_.end()), 95.0);

  const uint64_t total_pixels = static_cast<uint64_t>(depth.rows) * static_cast<uint64_t>(depth.cols);
  const int roi_width = std::max(1, static_cast<int>(std::round(depth.cols * kCenterRoiRatio)));
  const int roi_height = std::max(1, static_cast<int>(std::round(depth.rows * kCenterRoiRatio)));
  const int roi_x0 = std::max(0, (depth.cols - roi_width) / 2);
  const int roi_y0 = std::max(0, (depth.rows - roi_height) / 2);
  const uint64_t roi_pixels = static_cast<uint64_t>(roi_width) * static_cast<uint64_t>(roi_height);

  std::vector<float> valid_depths;
  valid_depths.reserve(static_cast<std::size_t>(total_pixels / 4U));

  for (int row = 0; row < depth.rows; ++row) {
    const auto * depth_ptr = depth.ptr<float>(row);
    for (int col = 0; col < depth.cols; ++col) {
      const float value = depth_ptr[col];
      if (!std::isfinite(value)) {
        continue;
      }
      ++metrics.valid_pixels_1m;
      valid_depths.push_back(value);
      if (row >= roi_y0 && row < (roi_y0 + roi_height) && col >= roi_x0 && col < (roi_x0 + roi_width)) {
        metrics.coverage_ratio_center_roi += 1.0;
      }
    }
  }

  metrics.valid_ratio_1m = total_pixels > 0U ?
    static_cast<double>(metrics.valid_pixels_1m) / static_cast<double>(total_pixels) : 0.0;
  metrics.coverage_ratio_center_roi = roi_pixels > 0U ?
    metrics.coverage_ratio_center_roi / static_cast<double>(roi_pixels) : 0.0;

  if (!valid_depths.empty()) {
    const std::size_t median_index = valid_depths.size() / 2U;
    std::nth_element(
      valid_depths.begin(),
      valid_depths.begin() + static_cast<std::ptrdiff_t>(median_index),
      valid_depths.end());
    metrics.median_depth_1m = static_cast<double>(valid_depths[median_index]);
  }

  return metrics;
}

std::string CudaStereoDepthNode::append_quality_metrics(const std::string & detail) const
{
  if (!quality_metrics_initialized_) {
    return detail;
  }

  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  stream << detail
         << " valid_ratio_1m=" << last_quality_metrics_.valid_ratio_1m
         << " valid_pixels_1m=" << last_quality_metrics_.valid_pixels_1m
         << " coverage_ratio_center_roi=" << last_quality_metrics_.coverage_ratio_center_roi
         << " median_depth_1m=" << compact_metric(last_quality_metrics_.median_depth_1m, 3)
         << " p95_processing_ms=" << compact_metric(last_quality_metrics_.p95_processing_ms, 2);
  return stream.str();
}

void CudaStereoDepthNode::on_timer()
{
  const int64_t timer_start_ns = now().nanoseconds();
  std::deque<FrameBundle> left_frames;
  std::deque<FrameBundle> right_frames;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prune_queues_locked(timer_start_ns);
    left_frames = left_frames_;
    right_frames = right_frames_;
  }

  PairSelection selection = select_best_pair(
    left_frames, right_frames, timer_start_ns);
  if (!selection.left.has_value() || !selection.right.has_value()) {
    switch (selection.state) {
      case DepthStreamState::kMissingInput:
        ++missing_input_count_;
        break;
      case DepthStreamState::kPairOutOfWindow:
        ++pair_out_of_window_count_;
        break;
      case DepthStreamState::kStaleFrame:
        ++stale_frame_count_;
        break;
      case DepthStreamState::kProcessingOverrun:
      case DepthStreamState::kOk:
      default:
        break;
    }
    if (selection.best_diff_ns >= 0) {
      last_pair_diff_ms_ = static_cast<double>(selection.best_diff_ns) / 1e6;
    }
    publish_state(selection.state, selection.detail);
    maybe_log_stats(timer_start_ns);
    return;
  }

  FrameBundle left = *selection.left;
  FrameBundle right = *selection.right;
  if (left.width != right.width || left.height != right.height) {
    ++missing_input_count_;
    publish_state(
      DepthStreamState::kMissingInput,
      "image_size_mismatch left=" + std::to_string(left.width) + "x" + std::to_string(left.height) +
      " right=" + std::to_string(right.width) + "x" + std::to_string(right.height));
    mark_pair_processed(left.seq, right.seq, timer_start_ns);
    maybe_log_stats(timer_start_ns);
    return;
  }

  if (calibration_.validated &&
    (left.width != calibration_.image_size.width || left.height != calibration_.image_size.height))
  {
    ++calibration_mismatch_count_;
    publish_state(
      DepthStreamState::kMissingInput,
      "validated_calibration_resolution_mismatch calibration=" +
      std::to_string(calibration_.image_size.width) + "x" +
      std::to_string(calibration_.image_size.height) + " runtime=" +
      std::to_string(left.width) + "x" + std::to_string(left.height));
    mark_pair_processed(left.seq, right.seq, timer_start_ns);
    maybe_log_stats(timer_start_ns);
    return;
  }

  ensure_rectify_maps(left.width, left.height);

  cv::cuda::GpuMat left_raw_gpu;
  cv::cuda::GpuMat right_raw_gpu;
  cv::cuda::GpuMat left_gray_gpu;
  cv::cuda::GpuMat right_gray_gpu;
  cv::cuda::GpuMat left_rect_gpu;
  cv::cuda::GpuMat right_rect_gpu;
  cv::cuda::GpuMat left_filtered_gpu;
  cv::cuda::GpuMat right_filtered_gpu;
  cv::cuda::GpuMat disparity_gpu;

  left_raw_gpu.upload(left.image);
  right_raw_gpu.upload(right.image);

  if (left.encoding == "mono8" || left.encoding == "8UC1") {
    left_gray_gpu = left_raw_gpu;
  } else if (left.encoding == "bgr8") {
    cv::cuda::cvtColor(left_raw_gpu, left_gray_gpu, cv::COLOR_BGR2GRAY);
  } else if (left.encoding == "rgb8") {
    cv::cuda::cvtColor(left_raw_gpu, left_gray_gpu, cv::COLOR_RGB2GRAY);
  } else {
    throw std::runtime_error("Unsupported left encoding: " + left.encoding);
  }

  if (right.encoding == "mono8" || right.encoding == "8UC1") {
    right_gray_gpu = right_raw_gpu;
  } else if (right.encoding == "bgr8") {
    cv::cuda::cvtColor(right_raw_gpu, right_gray_gpu, cv::COLOR_BGR2GRAY);
  } else if (right.encoding == "rgb8") {
    cv::cuda::cvtColor(right_raw_gpu, right_gray_gpu, cv::COLOR_RGB2GRAY);
  } else {
    throw std::runtime_error("Unsupported right encoding: " + right.encoding);
  }

  cv::cuda::remap(left_gray_gpu, left_rect_gpu, left_map1_gpu_, left_map2_gpu_, cv::INTER_LINEAR);
  cv::cuda::remap(right_gray_gpu, right_rect_gpu, right_map1_gpu_, right_map2_gpu_, cv::INTER_LINEAR);
  median_filter_->apply(left_rect_gpu, left_filtered_gpu);
  median_filter_->apply(right_rect_gpu, right_filtered_gpu);
  stereo_bm_->compute(left_filtered_gpu, right_filtered_gpu, disparity_gpu);

  cv::Mat left_rect;
  if (enable_wls_filter_ || publish_debug_rect_) {
    left_rect_gpu.download(left_rect);
  }
  cv::Mat disparity = compute_filtered_disparity(
    left_filtered_gpu,
    right_filtered_gpu,
    disparity_gpu,
    left_rect);

  cv::Mat depth(disparity.rows, disparity.cols, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  for (int row = 0; row < disparity.rows; ++row) {
    const auto * disp_ptr = disparity.ptr<float>(row);
    auto * depth_ptr = depth.ptr<float>(row);
    for (int col = 0; col < disparity.cols; ++col) {
      const float disparity_value = disp_ptr[col];
      if (disparity_value <= static_cast<float>(min_disparity_) || disparity_value <= 0.0F) {
        continue;
      }
      const float depth_value = static_cast<float>(
        (calibration_.fx * calibration_.baseline_m) / static_cast<double>(disparity_value));
      if (depth_value < static_cast<float>(min_depth_m_) || depth_value > static_cast<float>(max_depth_m_)) {
        continue;
      }
      depth_ptr[col] = depth_value;
    }
  }

  const rclcpp::Time stamp = left.stamp > right.stamp ? left.stamp : right.stamp;
  disparity_pub_->publish(make_image_msg(disparity, stamp, left.frame_id, "32FC1"));
  depth_pub_->publish(make_image_msg(depth, stamp, left.frame_id, "32FC1"));
  left_rect_info_pub_->publish(make_rectified_left_camera_info(stamp, left.frame_id));

  if (publish_debug_rect_) {
    cv::Mat right_rect;
    right_rect_gpu.download(right_rect);
    left_rect_pub_->publish(make_image_msg(left_rect, stamp, left.frame_id, "mono8"));
    right_rect_pub_->publish(make_image_msg(right_rect, stamp, right.frame_id, "mono8"));
  }

  if (publish_debug_mask_) {
    const cv::Mat valid_mask = valid_mask_from_depth(depth);
    valid_mask_pub_->publish(make_image_msg(valid_mask, stamp, left.frame_id, "mono8"));
  }

  ++published_pairs_;
  last_pair_diff_ms_ = static_cast<double>(selection.best_diff_ns) / 1e6;
  pair_diff_history_ms_.push_back(last_pair_diff_ms_);
  if (pair_diff_history_ms_.size() > 120U) {
    pair_diff_history_ms_.pop_front();
  }
  mark_pair_processed(left.seq, right.seq, now().nanoseconds());

  const double processing_ms =
    static_cast<double>(now().nanoseconds() - timer_start_ns) / 1e6;
  last_processing_ms_ = processing_ms;
  last_quality_metrics_ = compute_depth_quality_metrics(depth, processing_ms);
  quality_metrics_initialized_ = true;
  const bool processing_overrun =
    processing_ms > ((static_cast<double>(publish_period_ns_) * processing_overrun_factor_) / 1e6);
  if (processing_overrun) {
    ++processing_overrun_count_;
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(2);
    stream << "processing_ms=" << processing_ms
           << " budget_ms=" << ((static_cast<double>(publish_period_ns_) * processing_overrun_factor_) / 1e6)
           << " pair_diff_ms=" << last_pair_diff_ms_
           << " calibration_id=" << calibration_.calibration_id
           << " validated=" << (calibration_.validated ? "true" : "false");
    publish_state(DepthStreamState::kProcessingOverrun, stream.str());
  } else {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(2);
    stream << "processing_ms=" << processing_ms
           << " pair_diff_ms=" << last_pair_diff_ms_
           << " left_seq=" << left.seq
           << " right_seq=" << right.seq
           << " calibration_id=" << calibration_.calibration_id
           << " validated=" << (calibration_.validated ? "true" : "false");
    publish_state(DepthStreamState::kOk, stream.str());
  }
  maybe_log_stats(now().nanoseconds());
}

}  // namespace deyes_capture_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<deyes_capture_cpp::CudaStereoDepthNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
