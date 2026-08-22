#pragma once

#include <cstdint>
#include <string_view>

#include "deyes_capture_cpp/depth_projection.hpp"

namespace deyes_capture_cpp
{

struct StereoPairContractInput
{
  int64_t depth_stamp_ns{0};
  int64_t camera_info_stamp_ns{0};
  uint32_t depth_width{0};
  uint32_t depth_height{0};
  uint32_t camera_info_width{0};
  uint32_t camera_info_height{0};
  std::string_view depth_frame_id;
  std::string_view camera_info_frame_id;
  std::string_view depth_encoding;
  uint8_t depth_is_bigendian{0};
  uint32_t depth_step{0};
  uint64_t depth_data_size{0};
  RectifiedProjection projection;
};

struct ContractValidation
{
  bool accepted{false};
  const char * detail{"unknown"};
};

inline bool valid_calibration_identity(bool calibration_validated, std::string_view calibration_id)
{
  return !calibration_validated || (!calibration_id.empty() && calibration_id != "unassigned");
}

inline ContractValidation validate_stereo_pair_contract(const StereoPairContractInput & input)
{
  if (input.depth_stamp_ns != input.camera_info_stamp_ns) {
    return {false, "header_stamp_mismatch"};
  }
  if (input.depth_width == 0U || input.depth_height == 0U ||
    input.depth_width != input.camera_info_width ||
    input.depth_height != input.camera_info_height)
  {
    return {false, "image_camera_info_dimension_mismatch"};
  }
  if (input.depth_frame_id.empty() || input.depth_frame_id != input.camera_info_frame_id) {
    return {false, "image_camera_info_frame_id_mismatch"};
  }
  if (input.depth_encoding != "32FC1") {
    return {false, "depth_encoding_must_be_32FC1"};
  }
  if (input.depth_is_bigendian != 0U) {
    return {false, "big_endian_32FC1_not_supported"};
  }
  const uint64_t minimum_step = static_cast<uint64_t>(input.depth_width) * sizeof(float);
  const uint64_t expected_data_size = static_cast<uint64_t>(input.depth_step) * input.depth_height;
  if (input.depth_step < minimum_step || input.depth_data_size != expected_data_size) {
    return {false, "depth_step_or_data_length_invalid"};
  }
  if (!valid_projection(input.projection)) {
    return {false, "rectified_camera_info_projection_invalid"};
  }
  return {true, "ok"};
}

}  // namespace deyes_capture_cpp
