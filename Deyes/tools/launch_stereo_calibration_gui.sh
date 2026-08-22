#!/usr/bin/env bash
# Installed desktop launcher.  ``install_stereo_calibration_desktop.sh`` writes
# the resolved package prefix, so this remains valid across clean workspaces.
set -euo pipefail
_ld="${LD_LIBRARY_PATH:-}"
export LD_LIBRARY_PATH="$(printf '%s\n' "$_ld" | tr ':' '\n' | awk 'tolower($0) !~ /opencv.*cuda.*\/lib$/ && $0 { out = out (out ? ":" : "") $0 } END { printf "%s", out }')"
_config="${XDG_CONFIG_HOME:-${HOME}/.config}/deyes/stereo_calibration.env"
if [[ ! -r "$_config" ]]; then
  printf 'Mercury X1 stereo calibration is not installed for this user. Run install_stereo_calibration_desktop.sh from a sourced Deyes workspace.\n' >&2
  exit 2
fi
# shellcheck disable=SC1090
source "$_config"
_ros_distro="${ROS_DISTRO:-galactic}"
source "/opt/ros/${_ros_distro}/setup.bash"
if [[ -z "${DEYES_PACKAGE_PREFIX:-}" || ! -r "${DEYES_PACKAGE_PREFIX}/local_setup.bash" ]]; then
  printf 'Configured Deyes package prefix is missing: %s\n' "${DEYES_PACKAGE_PREFIX:-unset}" >&2
  exit 2
fi
source "${DEYES_PACKAGE_PREFIX}/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General><Discovery><Peers><Peer Address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
exec "${DEYES_PACKAGE_PREFIX}/lib/deyes_stereo/stereo_calibration_gui" "$@"
