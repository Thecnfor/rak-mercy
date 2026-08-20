#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace deyes_capture_cpp
{

struct RectifiedProjection
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

struct PointXYZ
{
  float x{std::numeric_limits<float>::quiet_NaN()};
  float y{std::numeric_limits<float>::quiet_NaN()};
  float z{std::numeric_limits<float>::quiet_NaN()};
};

struct OrganizedCloudLayout
{
  uint32_t width{0};
  uint32_t height{0};
  uint32_t point_step{12};
  uint32_t row_step{0};
};

inline bool has_valid_points(uint64_t valid_points)
{
  return valid_points > 0U;
}

inline bool valid_projection(const RectifiedProjection & projection)
{
  return std::isfinite(projection.fx) && std::isfinite(projection.fy) &&
         std::isfinite(projection.cx) && std::isfinite(projection.cy) &&
         projection.fx > 0.0 && projection.fy > 0.0;
}

inline OrganizedCloudLayout organized_cloud_layout(
  uint32_t input_width, uint32_t input_height, uint32_t sample_step)
{
  OrganizedCloudLayout layout;
  if (input_width == 0U || input_height == 0U || sample_step == 0U) {
    return layout;
  }
  layout.width = (input_width + sample_step - 1U) / sample_step;
  layout.height = (input_height + sample_step - 1U) / sample_step;
  layout.row_step = layout.width * layout.point_step;
  return layout;
}

inline PointXYZ project_depth_pixel(
  uint32_t pixel_u, uint32_t pixel_v, float depth_m,
  const RectifiedProjection & projection, float min_depth_m, float max_depth_m)
{
  PointXYZ result;
  if (!valid_projection(projection) || !std::isfinite(min_depth_m) || !std::isfinite(max_depth_m) ||
    min_depth_m <= 0.0F || max_depth_m < min_depth_m || !std::isfinite(depth_m) ||
    depth_m < min_depth_m || depth_m > max_depth_m)
  {
    return result;
  }
  result.z = depth_m;
  result.x = static_cast<float>((static_cast<double>(pixel_u) - projection.cx) * depth_m / projection.fx);
  result.y = static_cast<float>((static_cast<double>(pixel_v) - projection.cy) * depth_m / projection.fy);
  if (!std::isfinite(result.x) || !std::isfinite(result.y)) {
    return PointXYZ{};
  }
  return result;
}

}  // namespace deyes_capture_cpp
