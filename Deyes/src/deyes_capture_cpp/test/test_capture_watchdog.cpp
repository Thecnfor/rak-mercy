#include <limits>

#include <gtest/gtest.h>

#include "deyes_capture_cpp/capture_watchdog.hpp"

TEST(CaptureWatchdog, DoesNotRestartFreshStream)
{
  EXPECT_FALSE(deyes_capture_cpp::capture_stream_stalled(true, 0.03, 2.0));
  EXPECT_FALSE(deyes_capture_cpp::capture_fail_fast_due(
      true, 0.03, true, 0.04, 2.0, 20.0, 8.0));
}

TEST(CaptureWatchdog, DetectsNoFirstFrameAndStalledStream)
{
  EXPECT_TRUE(deyes_capture_cpp::capture_stream_stalled(false, 0.0, 2.0));
  EXPECT_TRUE(deyes_capture_cpp::capture_stream_stalled(
      true, std::numeric_limits<double>::infinity(), 2.0));
  EXPECT_TRUE(deyes_capture_cpp::capture_stream_stalled(true, 2.01, 2.0));
}

TEST(CaptureWatchdog, FailsFastWhenEitherCameraStopsAfterStartup)
{
  EXPECT_TRUE(deyes_capture_cpp::capture_fail_fast_due(
      true, 4.0, true, 0.03, 2.0, 20.0, 8.0));
  EXPECT_TRUE(deyes_capture_cpp::capture_fail_fast_due(
      true, 0.03, false, 0.0, 2.0, 20.0, 8.0));
}

TEST(CaptureWatchdog, SuppressesRestartDuringStartupGrace)
{
  EXPECT_FALSE(deyes_capture_cpp::capture_fail_fast_due(
      false, 0.0, false, 0.0, 2.0, 7.99, 8.0));
  EXPECT_TRUE(deyes_capture_cpp::capture_fail_fast_due(
      false, 0.0, false, 0.0, 2.0, 8.0, 8.0));
}
