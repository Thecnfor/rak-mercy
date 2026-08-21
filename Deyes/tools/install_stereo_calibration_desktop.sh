#!/usr/bin/env bash
# Run after sourcing the clean colcon install that contains deyes_stereo.
set -euo pipefail
_prefix="$(ros2 pkg prefix deyes_stereo)"
_share="${_prefix}/share/deyes_stereo/tools"
_launcher="${HOME}/.local/bin/deyes-stereo-calibration-gui"
_desktop="${HOME}/Desktop/mercury-x1-stereo-calibration.desktop"
_config="${XDG_CONFIG_HOME:-${HOME}/.config}/deyes/stereo_calibration.env"
test -x "${_prefix}/lib/deyes_stereo/stereo_calibration_gui"
test -r "${_share}/launch_stereo_calibration_gui.sh"
test -r "${_share}/mercury-x1-stereo-calibration.desktop"
mkdir -p "$(dirname "$_launcher")" "$(dirname "$_config")" "$(dirname "$_desktop")"
install -m 0755 "${_share}/launch_stereo_calibration_gui.sh" "$_launcher"
printf 'DEYES_PACKAGE_PREFIX=%q\n' "$_prefix" > "$_config"
sed "s|@DEYES_GUI_LAUNCHER@|${_launcher}|" "${_share}/mercury-x1-stereo-calibration.desktop" > "$_desktop"
chmod 0755 "$_desktop"
printf 'Installed stereo calibration desktop entry: %s\n' "$_desktop"
