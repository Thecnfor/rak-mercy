#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include "deyes_capture_cpp/depth_projection.hpp"
#include "deyes_capture_cpp/stereo_pair_contract.hpp"

namespace
{

TEST(DepthProjection, ProjectsExactRectifiedXYZ)
{
  const deyes_capture_cpp::RectifiedProjection projection{100.0, 200.0, 10.0, 20.0};
  const auto point = deyes_capture_cpp::project_depth_pixel(30U, 60U, 0.5F, projection, 0.2F, 1.0F);
  EXPECT_FLOAT_EQ(point.x, 0.1F);
  EXPECT_FLOAT_EQ(point.y, 0.1F);
  EXPECT_FLOAT_EQ(point.z, 0.5F);
}

TEST(DepthProjection, MakesNonFiniteAndOutOfRangeDepthNan)
{
  const deyes_capture_cpp::RectifiedProjection projection{100.0, 100.0, 0.0, 0.0};
  for (const float depth : {NAN, INFINITY, 0.19F, 1.01F}) {
    const auto point = deyes_capture_cpp::project_depth_pixel(0U, 0U, depth, projection, 0.2F, 1.0F);
    EXPECT_TRUE(std::isnan(point.x));
    EXPECT_TRUE(std::isnan(point.y));
    EXPECT_TRUE(std::isnan(point.z));
  }
}

TEST(DepthProjection, RejectsInvalidProjectionRange)
{
  const deyes_capture_cpp::RectifiedProjection projection{100.0, 100.0, 0.0, 0.0};
  const auto nonfinite_min = deyes_capture_cpp::project_depth_pixel(0U, 0U, 0.5F, projection, NAN, 1.0F);
  const auto inverted_range = deyes_capture_cpp::project_depth_pixel(0U, 0U, 0.5F, projection, 1.0F, 0.2F);
  EXPECT_TRUE(std::isnan(nonfinite_min.z));
  EXPECT_TRUE(std::isnan(inverted_range.z));
}

TEST(DepthProjection, SampledLayoutStaysOrganized)
{
  const auto layout = deyes_capture_cpp::organized_cloud_layout(5U, 3U, 2U);
  EXPECT_EQ(layout.width, 3U);
  EXPECT_EQ(layout.height, 2U);
  EXPECT_EQ(layout.point_step, 12U);
  EXPECT_EQ(layout.row_step, 36U);
}

TEST(DepthProjection, ZeroValidPointsIsNotASuccessState)
{
  EXPECT_FALSE(deyes_capture_cpp::has_valid_points(0U));
  EXPECT_TRUE(deyes_capture_cpp::has_valid_points(1U));
}

TEST(DepthProjection, SamplingMapsOutputPixelToInputStride)
{
  const deyes_capture_cpp::RectifiedProjection projection{10.0, 10.0, 0.0, 0.0};
  // Output pixel (2,1) for step=2 maps to input pixel (4,2).
  const auto point = deyes_capture_cpp::project_depth_pixel(4U, 2U, 0.5F, projection, 0.2F, 1.0F);
  EXPECT_FLOAT_EQ(point.x, 0.2F);
  EXPECT_FLOAT_EQ(point.y, 0.1F);
}

deyes_capture_cpp::StereoPairContractInput valid_contract()
{
  deyes_capture_cpp::StereoPairContractInput input;
  input.depth_stamp_ns = 123;
  input.camera_info_stamp_ns = 123;
  input.depth_width = 640U;
  input.depth_height = 360U;
  input.camera_info_width = 640U;
  input.camera_info_height = 360U;
  input.depth_frame_id = "left_camera_optical_frame";
  input.camera_info_frame_id = "left_camera_optical_frame";
  input.depth_encoding = "32FC1";
  input.depth_step = 640U * sizeof(float);
  input.depth_data_size = static_cast<uint64_t>(input.depth_step) * input.depth_height;
  input.projection = {500.0, 500.0, 320.0, 180.0};
  return input;
}

TEST(StereoPairContract, AcceptsOnlyExactMatchingRectifiedInputs)
{
  EXPECT_TRUE(deyes_capture_cpp::validate_stereo_pair_contract(valid_contract()).accepted);
}

TEST(StereoPairContract, RejectsStampSizeFrameEncodingStepDataAndProjection)
{
  auto input = valid_contract();
  input.camera_info_stamp_ns++;
  EXPECT_STREQ(deyes_capture_cpp::validate_stereo_pair_contract(input).detail, "header_stamp_mismatch");
  input = valid_contract();
  input.camera_info_width++;
  EXPECT_STREQ(
    deyes_capture_cpp::validate_stereo_pair_contract(input).detail,
    "image_camera_info_dimension_mismatch");
  input = valid_contract();
  input.camera_info_frame_id = "right_camera_optical_frame";
  EXPECT_STREQ(deyes_capture_cpp::validate_stereo_pair_contract(input).detail, "image_camera_info_frame_id_mismatch");
  input = valid_contract();
  input.depth_encoding = "16UC1";
  EXPECT_STREQ(deyes_capture_cpp::validate_stereo_pair_contract(input).detail, "depth_encoding_must_be_32FC1");
  input = valid_contract();
  input.depth_step--;
  EXPECT_STREQ(deyes_capture_cpp::validate_stereo_pair_contract(input).detail, "depth_step_or_data_length_invalid");
  input = valid_contract();
  input.depth_data_size--;
  EXPECT_STREQ(deyes_capture_cpp::validate_stereo_pair_contract(input).detail, "depth_step_or_data_length_invalid");
  input = valid_contract();
  input.projection.fx = 0.0;
  EXPECT_STREQ(
    deyes_capture_cpp::validate_stereo_pair_contract(input).detail,
    "rectified_camera_info_projection_invalid");
}

TEST(StereoPairContract, ValidatedModeRequiresPhysicalCalibrationIdentity)
{
  EXPECT_TRUE(deyes_capture_cpp::valid_calibration_identity(false, "unassigned"));
  EXPECT_FALSE(deyes_capture_cpp::valid_calibration_identity(true, ""));
  EXPECT_FALSE(deyes_capture_cpp::valid_calibration_identity(true, "unassigned"));
  EXPECT_TRUE(deyes_capture_cpp::valid_calibration_identity(true, "x1-pair-20260820"));
}

}  // namespace
