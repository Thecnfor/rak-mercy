#pragma once

#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace deyes_capture_cpp
{

class StereoPointCloudNode : public rclcpp::Node
{
public:
  explicit StereoPointCloudNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_depth(const sensor_msgs::msg::Image::SharedPtr msg);
  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void on_timer();
  void trim_queues_locked();
  bool process_pair(
    const sensor_msgs::msg::Image & depth,
    const sensor_msgs::msg::CameraInfo & info,
    std::string * rejection_detail,
    uint64_t * valid_points);
  void publish_status(uint8_t level, const std::string & state, const std::string & detail);
  double processing_p95_ms() const;

  std::mutex mutex_;
  std::deque<sensor_msgs::msg::Image::SharedPtr> depth_queue_;
  std::deque<sensor_msgs::msg::CameraInfo::SharedPtr> info_queue_;
  std::vector<double> processing_history_ms_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string depth_topic_;
  std::string rectified_camera_info_topic_;
  std::string points_topic_;
  std::string status_topic_;
  std::string calibration_id_;
  bool calibration_validated_{false};
  double min_depth_m_{0.20};
  double max_depth_m_{1.00};
  double publish_period_sec_{0.07};
  int sample_step_{2};
  int queue_size_{8};

  uint64_t received_depth_{0};
  uint64_t received_info_{0};
  uint64_t dropped_depth_{0};
  uint64_t dropped_info_{0};
  uint64_t queue_overflow_depth_{0};
  uint64_t queue_overflow_info_{0};
  uint64_t rejected_pairs_{0};
  uint64_t published_clouds_{0};
  uint64_t last_valid_points_{0};
  uint64_t last_total_points_{0};
  double last_processing_ms_{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace deyes_capture_cpp
