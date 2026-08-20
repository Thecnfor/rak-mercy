#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

namespace deyes_capture_cpp
{

struct StereoCalibration
{
  cv::Size image_size;
  cv::Mat k1;
  cv::Mat d1;
  cv::Mat k2;
  cv::Mat d2;
  cv::Mat r;
  cv::Mat t;
  double baseline_m{0.0};
  double fx{0.0};
};

struct FrameBundle
{
  rclcpp::Time stamp;
  std::string frame_id;
  std::string encoding;
  cv::Mat image;
  int width{0};
  int height{0};
  uint64_t seq{0};
};

struct DepthQualityMetrics
{
  double valid_ratio_1m{0.0};
  uint64_t valid_pixels_1m{0};
  double coverage_ratio_center_roi{0.0};
  double median_depth_1m{std::numeric_limits<double>::quiet_NaN()};
  double p95_processing_ms{std::numeric_limits<double>::quiet_NaN()};
};

enum class DepthStreamState
{
  kOk,
  kMissingInput,
  kPairOutOfWindow,
  kStaleFrame,
  kProcessingOverrun,
};

class CudaStereoDepthNode : public rclcpp::Node
{
public:
  explicit CudaStereoDepthNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct PairSelection
  {
    DepthStreamState state{DepthStreamState::kMissingInput};
    std::optional<FrameBundle> left;
    std::optional<FrameBundle> right;
    std::string detail;
    int64_t best_diff_ns{-1};
  };

  StereoCalibration load_stereo_calibration(const std::string & calib_path) const;
  cv::Mat image_msg_to_mat(const sensor_msgs::msg::Image & msg) const;
  sensor_msgs::msg::Image make_image_msg(
    const cv::Mat & frame,
    const rclcpp::Time & stamp,
    const std::string & frame_id,
    const std::string & encoding) const;
  void enqueue_frame(std::deque<FrameBundle> & queue, FrameBundle && bundle);
  void prune_queues_locked(int64_t now_ns);
  PairSelection select_best_pair(
    const std::deque<FrameBundle> & left_frames,
    const std::deque<FrameBundle> & right_frames,
    bool have_left_info,
    bool have_right_info,
    int64_t now_ns) const;
  void mark_pair_processed(uint64_t left_seq, uint64_t right_seq, int64_t now_ns);
  void publish_state(DepthStreamState state, const std::string & detail);
  void maybe_log_stats(int64_t now_ns);
  static const char * state_to_string(DepthStreamState state);
  void ensure_rectify_maps(int width, int height);
  void on_left_image(const sensor_msgs::msg::Image::SharedPtr msg);
  void on_right_image(const sensor_msgs::msg::Image::SharedPtr msg);
  void on_left_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void on_right_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void on_timer();
  cv::Mat valid_mask_from_depth(const cv::Mat & depth) const;
  cv::Mat disparity_to_float32(const cv::Mat & disparity) const;
  cv::Mat compute_filtered_disparity(
    const cv::cuda::GpuMat & left_filtered_gpu,
    const cv::cuda::GpuMat & right_filtered_gpu,
    const cv::cuda::GpuMat & disparity_left_gpu,
    const cv::Mat & left_rect_cpu);
  DepthQualityMetrics compute_depth_quality_metrics(const cv::Mat & depth, double processing_ms);
  std::string append_quality_metrics(const std::string & detail) const;
  void restart_wls_filter();

  mutable std::mutex mutex_;

  StereoCalibration calibration_;
  std::deque<FrameBundle> left_frames_;
  std::deque<FrameBundle> right_frames_;
  std::optional<sensor_msgs::msg::CameraInfo> left_info_;
  std::optional<sensor_msgs::msg::CameraInfo> right_info_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rect_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_rect_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr valid_mask_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_detail_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv::Ptr<cv::cuda::Filter> median_filter_;
  cv::Ptr<cv::cuda::StereoBM> stereo_bm_;
  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_;

  cv::cuda::GpuMat left_map1_gpu_;
  cv::cuda::GpuMat left_map2_gpu_;
  cv::cuda::GpuMat right_map1_gpu_;
  cv::cuda::GpuMat right_map2_gpu_;
  cv::Size rectify_size_;

  int64_t max_sync_diff_ns_{3'000'000};
  int64_t frame_stale_ns_{200'000'000};
  int64_t publish_period_ns_{33'333'333};
  double min_depth_m_{0.2};
  double max_depth_m_{1.5};
  double processing_overrun_factor_{1.0};
  bool enable_wls_filter_{true};
  double wls_lambda_{8000.0};
  double wls_sigma_color_{2.0};
  int min_disparity_{0};
  int texture_threshold_{0};
  int uniqueness_ratio_{0};
  int speckle_window_size_{0};
  int speckle_range_{0};
  int disp12_max_diff_{0};
  bool publish_debug_rect_{true};
  bool publish_debug_mask_{true};
  std::size_t frame_queue_size_{8};
  double log_stats_period_sec_{2.0};
  double last_stats_log_sec_{0.0};
  double last_pair_diff_ms_{0.0};
  double last_processing_ms_{0.0};
  uint64_t next_left_seq_{1};
  uint64_t next_right_seq_{1};
  uint64_t last_processed_left_seq_{0};
  uint64_t last_processed_right_seq_{0};
  uint64_t published_pairs_{0};
  uint64_t missing_input_count_{0};
  uint64_t pair_out_of_window_count_{0};
  uint64_t stale_frame_count_{0};
  uint64_t processing_overrun_count_{0};
  uint64_t last_logged_published_pairs_{0};
  uint64_t last_logged_missing_input_count_{0};
  uint64_t last_logged_pair_out_of_window_count_{0};
  uint64_t last_logged_stale_frame_count_{0};
  uint64_t last_logged_processing_overrun_count_{0};
  DepthStreamState current_state_{DepthStreamState::kMissingInput};
  std::string current_state_detail_;
  bool state_initialized_{false};
  DepthQualityMetrics last_quality_metrics_;
  bool quality_metrics_initialized_{false};
  std::deque<double> pair_diff_history_ms_;
  std::deque<double> processing_history_ms_;
};

}  // namespace deyes_capture_cpp
