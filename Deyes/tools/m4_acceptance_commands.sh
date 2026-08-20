#!/usr/bin/env bash
# Prints, but does not start, the exact Phase 4 acceptance commands.
set -euo pipefail

: "${TEMP_ROOT:?Set TEMP_ROOT=/home/<robot-user>/temp/deyes outside the repository.}"
SESSION_ID="${SESSION_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
SESSION_DIR="${TEMP_ROOT}/acceptance/${SESSION_ID}"

cat <<EOF
Create the external evidence directory first:
  mkdir -p ${SESSION_DIR}/{rosbag,images,reports}

1. Start formal 640x360@30 debug chain only (physical YAML must remain validated=false until M2 passes):
  ros2 launch deyes_bringup imx219_stereo.launch.py use_cpp_capture:=true width:=640 height:=360 fps:=30 enable_cuda_depth:=true enable_pointcloud:=true pointcloud_calibration_validated:=false

2. In a separate terminal record the required 10-minute evidence (all output is under temp):
  ros2 bag record -o ${SESSION_DIR}/rosbag/runtime_10min /x1/left_camera/image_raw /x1/right_camera/image_raw /x1/stereo/depth /x1/stereo/points /x1/stereo/points_status /x1/stereo/pair_diagnostics /cuda_stereo_depth_node/status /cuda_stereo_depth_node/status_detail
  ros2 run deyes_stereo runtime_acceptance_monitor --output-dir ${SESSION_DIR}/reports --duration-sec 600 --rviz-check-file ${SESSION_DIR}/rviz_checks.json

3. Make truth_samples.csv with columns truth_m,measured_m,valid,plane_residual_m. For EACH of 0.30, 0.50, 0.80 and 1.00 m include at least 100 unfiltered frame samples. Measure from the left optical centre to the reference plane.
  ros2 run deyes_stereo stereo_acceptance truth --input ${SESSION_DIR}/truth_samples.csv --output-dir ${SESSION_DIR}/reports

4. Before the collector finishes, an operator must create ${SESSION_DIR}/rviz_checks.json with all three explicit checks. Missing/false checks make the report fail:
  {"flat_plane_has_no_obvious_warping_or_layering": true, "no_obvious_ghosting": true, "optical_axes_are_x_right_y_down_z_forward": true}

5. Re-evaluate a recorded runtime contract if needed:
  ros2 run deyes_stereo stereo_acceptance runtime --input ${SESSION_DIR}/runtime_metrics.json --output-dir ${SESSION_DIR}/reports

Operator must still record RViz checks: planar surface no visible bend/layering/ghosting, and optical axes X-right/Y-down/Z-forward. A report with validated=false must not be used for grasping.
EOF
