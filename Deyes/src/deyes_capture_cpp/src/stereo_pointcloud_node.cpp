#include "deyes_capture_cpp/stereo_pointcloud_node.hpp"

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

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "deyes_capture_cpp/depth_projection.hpp"
#include "deyes_capture_cpp/stereo_pair_contract.hpp"

namespace deyes_capture_cpp
{
namespace
{

constexpr std::size_t kProcessingHistoryLimit = 120U;

bool same_stamp(const builtin_interfaces::msg::Time & left, const builtin_interfaces::msg::Time & right)
{
  return left.sec == right.sec && left.nanosec == right.nanosec;
}

double percentile95(std::vector<double> values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t index = static_cast<std::size_t>(std::ceil(0.95 * values.size())) - 1U;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
  return values[index];
}

std::string as_string(uint64_t value)
{
  return std::to_string(static_cast<unsigned long long>(value));
}

std::string as_string(double value)
{
  if (!std::isfinite(value)) {
    return "nan";
  }
  std::ostringstream output;
  output.setf(std::ios::fixed);
  output.precision(2);
  output << value;
  return output.str();
}

sensor_msgs::msg::PointField make_point_field(
  const std::string & name, uint32_t offset, uint8_t datatype, uint32_t count)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = count;
  return field;
}

diagnostic_msgs::msg::KeyValue make_key_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = std::move(key);
  key_value.value = std::move(value);
  return key_value;
}

}  // namespace

StereoPointCloudNode::StereoPointCloudNode(const rclcpp::NodeOptions & options)
: Node("stereo_pointcloud_node", options)
{
  depth_topic_ = declare_parameter<std::string>("depth_topic", "/x1/stereo/depth");
  rectified_camera_info_topic_ = declare_parameter<std::string>(
    "rectified_camera_info_topic", "/x1/stereo/left/camera_info_rect");
  points_topic_ = declare_parameter<std::string>("points_topic", "/x1/stereo/points");
  status_topic_ = declare_parameter<std::string>("status_topic", "/x1/stereo/points_status");
  calibration_id_ = declare_parameter<std::string>("calibration_id", "unassigned");
  calibration_validated_ = declare_parameter<bool>("calibration_validated", false);
  min_depth_m_ = declare_parameter<double>("min_depth_m", 0.20);
  max_depth_m_ = declare_parameter<double>("max_depth_m", 1.00);
  publish_period_sec_ = declare_parameter<double>("publish_period_sec", 0.07);
  sample_step_ = declare_parameter<int>("sample_step", 2);
  queue_size_ = declare_parameter<int>("queue_size", 8);

  if (!std::isfinite(min_depth_m_) || !std::isfinite(max_depth_m_) ||
    !std::isfinite(publish_period_sec_) || min_depth_m_ <= 0.0 ||
    max_depth_m_ <= min_depth_m_ || publish_period_sec_ <= 0.0 ||
    sample_step_ <= 0 || queue_size_ <= 0)
  {
    throw std::invalid_argument("pointcloud parameters must define positive ranges, period, sample_step, and queue_size");
  }
  if (!valid_calibration_identity(calibration_validated_, calibration_id_)) {
    throw std::invalid_argument(
            "calibration_validated=true requires a non-empty calibration_id from physical calibration");
  }

  const auto sensor_qos = rclcpp::SensorDataQoS();
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, sensor_qos,
    std::bind(&StereoPointCloudNode::on_depth, this, std::placeholders::_1));
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    rectified_camera_info_topic_, sensor_qos,
    std::bind(&StereoPointCloudNode::on_camera_info, this, std::placeholders::_1));
  points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(points_topic_, sensor_qos);
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic_, 10);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(publish_period_sec_)),
    std::bind(&StereoPointCloudNode::on_timer, this));

  publish_status(
    calibration_validated_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    calibration_validated_ ? "waiting_for_pair" : "debug_only_waiting_for_pair",
    calibration_validated_ ? "awaiting exact depth/CameraInfo pair" :
    "calibration_validated=false; use=debug_rviz_only; grasp consumers must not consume this cloud");
}

void StereoPointCloudNode::on_depth(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++received_depth_;
  depth_queue_.push_back(msg);
  trim_queues_locked();
}

void StereoPointCloudNode::on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++received_info_;
  info_queue_.push_back(msg);
  trim_queues_locked();
}

void StereoPointCloudNode::trim_queues_locked()
{
  while (static_cast<int>(depth_queue_.size()) > queue_size_) {
    depth_queue_.pop_front();
    ++queue_overflow_depth_;
  }
  while (static_cast<int>(info_queue_.size()) > queue_size_) {
    info_queue_.pop_front();
    ++queue_overflow_info_;
  }
}

void StereoPointCloudNode::on_timer()
{
  sensor_msgs::msg::Image::SharedPtr depth;
  sensor_msgs::msg::CameraInfo::SharedPtr info;
  std::string waiting_detail;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!depth_queue_.empty() && !info_queue_.empty()) {
      const auto & queued_depth = depth_queue_.front();
      const auto & queued_info = info_queue_.front();
      if (same_stamp(queued_depth->header.stamp, queued_info->header.stamp)) {
        depth = queued_depth;
        info = queued_info;
        depth_queue_.pop_front();
        info_queue_.pop_front();
        break;
      }
      const rclcpp::Time depth_stamp(queued_depth->header.stamp);
      const rclcpp::Time info_stamp(queued_info->header.stamp);
      if (depth_stamp < info_stamp) {
        depth_queue_.pop_front();
        ++dropped_depth_;
        waiting_detail = "dropped_depth_timestamp_without_exact_camera_info";
      } else {
        info_queue_.pop_front();
        ++dropped_info_;
        waiting_detail = "dropped_camera_info_timestamp_without_exact_depth";
      }
    }
    if (!depth || !info) {
      if (waiting_detail.empty()) {
        waiting_detail = "waiting_for_exact_depth_camera_info_pair";
      }
    }
  }
  if (!depth || !info) {
    publish_status(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      calibration_validated_ ? "waiting_for_pair" : "debug_only_waiting_for_pair", waiting_detail);
    return;
  }

  const auto start = std::chrono::steady_clock::now();
  std::string rejection_detail;
  uint64_t valid_points = 0;
  if (!process_pair(*depth, *info, &rejection_detail, &valid_points)) {
    ++rejected_pairs_;
    publish_status(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "rejected_pair", rejection_detail);
    return;
  }
  const auto finish = std::chrono::steady_clock::now();
  const double processing_ms = std::chrono::duration<double, std::milli>(finish - start).count();
  last_processing_ms_ = processing_ms;
  processing_history_ms_.push_back(processing_ms);
  if (processing_history_ms_.size() > kProcessingHistoryLimit) {
    processing_history_ms_.erase(processing_history_ms_.begin());
  }
  ++published_clouds_;
  if (!has_valid_points(valid_points)) {
    std::ostringstream detail;
    detail.setf(std::ios::fixed);
    detail.precision(2);
    detail << "all sampled depth values were invalid or outside configured range [" << min_depth_m_
           << ", " << max_depth_m_ << "] m; cloud published as organized NaNs for RViz";
    publish_status(diagnostic_msgs::msg::DiagnosticStatus::WARN, "no_valid_points", detail.str());
    return;
  }
  publish_status(
    calibration_validated_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    calibration_validated_ ? "ok" : "debug_only",
    calibration_validated_ ? "rectified depth projected" :
    "calibration_validated=false; use=debug_rviz_only; grasp consumers must not consume this cloud");
}

bool StereoPointCloudNode::process_pair(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & info,
  std::string * rejection_detail, uint64_t * valid_points)
{
  const auto reject = [rejection_detail](const std::string & detail) {
      *rejection_detail = detail;
      return false;
    };
  const RectifiedProjection projection{info.p[0], info.p[5], info.p[2], info.p[6]};
  const StereoPairContractInput contract_input{
    rclcpp::Time(depth.header.stamp).nanoseconds(),
    rclcpp::Time(info.header.stamp).nanoseconds(),
    depth.width,
    depth.height,
    info.width,
    info.height,
    depth.header.frame_id,
    info.header.frame_id,
    depth.encoding,
    depth.is_bigendian,
    depth.step,
    static_cast<uint64_t>(depth.data.size()),
    projection};
  const ContractValidation contract = validate_stereo_pair_contract(contract_input);
  if (!contract.accepted) {
    return reject(contract.detail);
  }

  const auto layout = organized_cloud_layout(
    depth.width, depth.height, static_cast<uint32_t>(sample_step_));
  if (layout.width == 0U || layout.height == 0U) {
    return reject("organized_cloud_layout_invalid");
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = depth.header;
  cloud.height = layout.height;
  cloud.width = layout.width;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  cloud.point_step = layout.point_step;
  cloud.row_step = layout.row_step;
  cloud.fields = {
    make_point_field("x", 0U, sensor_msgs::msg::PointField::FLOAT32, 1U),
    make_point_field("y", 4U, sensor_msgs::msg::PointField::FLOAT32, 1U),
    make_point_field("z", 8U, sensor_msgs::msg::PointField::FLOAT32, 1U)};
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step) * cloud.height);

  uint64_t valid_point_count = 0;
  for (uint32_t output_v = 0; output_v < layout.height; ++output_v) {
    const uint32_t input_v = output_v * static_cast<uint32_t>(sample_step_);
    const auto * input_row = depth.data.data() + static_cast<std::size_t>(input_v) * depth.step;
    for (uint32_t output_u = 0; output_u < layout.width; ++output_u) {
      const uint32_t input_u = output_u * static_cast<uint32_t>(sample_step_);
      float depth_m = std::numeric_limits<float>::quiet_NaN();
      std::memcpy(&depth_m, input_row + static_cast<std::size_t>(input_u) * sizeof(float), sizeof(float));
      const PointXYZ point = project_depth_pixel(
        input_u, input_v, depth_m, projection,
        static_cast<float>(min_depth_m_), static_cast<float>(max_depth_m_));
      auto * output = cloud.data.data() + static_cast<std::size_t>(output_v) * cloud.row_step +
        static_cast<std::size_t>(output_u) * cloud.point_step;
      std::memcpy(output, &point.x, sizeof(float));
      std::memcpy(output + sizeof(float), &point.y, sizeof(float));
      std::memcpy(output + 2U * sizeof(float), &point.z, sizeof(float));
      if (std::isfinite(point.z)) {
        ++valid_point_count;
      }
    }
  }
  last_valid_points_ = valid_point_count;
  last_total_points_ = static_cast<uint64_t>(layout.width) * layout.height;
  *valid_points = last_valid_points_;
  points_pub_->publish(cloud);
  return true;
}

double StereoPointCloudNode::processing_p95_ms() const
{
  return percentile95(processing_history_ms_);
}

void StereoPointCloudNode::publish_status(
  uint8_t level, const std::string & state, const std::string & detail)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = get_name();
  status.hardware_id = "x1_stereo";
  status.message = state;
  const double coverage = last_total_points_ == 0U ? 0.0 :
    static_cast<double>(last_valid_points_) / static_cast<double>(last_total_points_);
  status.values = {
    make_key_value("state", state),
    make_key_value("detail", detail),
    make_key_value("valid_points", as_string(last_valid_points_)),
    make_key_value("total_points", as_string(last_total_points_)),
    make_key_value("coverage", as_string(coverage)),
    make_key_value("calibration_id", calibration_id_),
    make_key_value("calibration_validated", calibration_validated_ ? "true" : "false"),
    make_key_value("processing_ms", as_string(last_processing_ms_)),
    make_key_value("processing_p95_ms", as_string(processing_p95_ms())),
    make_key_value("received_depth", as_string(received_depth_)),
    make_key_value("received_camera_info", as_string(received_info_)),
    make_key_value("dropped_depth_timestamp", as_string(dropped_depth_)),
    make_key_value("dropped_camera_info_timestamp", as_string(dropped_info_)),
    make_key_value("queue_overflow_depth", as_string(queue_overflow_depth_)),
    make_key_value("queue_overflow_camera_info", as_string(queue_overflow_info_)),
    make_key_value("rejected_pairs", as_string(rejected_pairs_)),
    make_key_value("published_clouds", as_string(published_clouds_))};
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(std::move(status));
  status_pub_->publish(array);
}

}  // namespace deyes_capture_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<deyes_capture_cpp::StereoPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
