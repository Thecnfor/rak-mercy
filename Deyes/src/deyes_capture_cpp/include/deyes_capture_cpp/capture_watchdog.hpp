#pragma once

#include <cmath>

namespace deyes_capture_cpp
{

// This deliberately uses an age supplied by a monotonic clock.  Camera message
// stamps are wall-clock timestamps and may jump when NTP adjusts the Jetson.
inline bool capture_stream_stalled(
  bool has_received_frame,
  double last_receipt_age_sec,
  double stall_after_sec)
{
  return !has_received_frame ||
         !std::isfinite(last_receipt_age_sec) ||
         last_receipt_age_sec > stall_after_sec;
}

inline bool capture_fail_fast_due(
  bool left_has_received_frame,
  double left_receipt_age_sec,
  bool right_has_received_frame,
  double right_receipt_age_sec,
  double stall_after_sec,
  double seconds_since_start,
  double startup_grace_sec)
{
  return seconds_since_start >= startup_grace_sec &&
         (capture_stream_stalled(left_has_received_frame, left_receipt_age_sec, stall_after_sec) ||
         capture_stream_stalled(right_has_received_frame, right_receipt_age_sec, stall_after_sec));
}

}  // namespace deyes_capture_cpp
