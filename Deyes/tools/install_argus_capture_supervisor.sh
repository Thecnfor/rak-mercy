#!/usr/bin/env bash
# Install only on the Jetson, after a review.  This does not contact a robot
# and does not contain or request credentials.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SYSTEMD_DIR="${SCRIPT_DIR}/systemd"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo on the Jetson: sudo $0" >&2
  exit 64
fi

install -d -m 0755 /usr/local/lib/deyes /etc/systemd/system
install -m 0755 "${SYSTEMD_DIR}/deyes-stereo-capture-exec.sh" /usr/local/lib/deyes/
install -m 0755 "${SYSTEMD_DIR}/deyes-argus-recover.sh" /usr/local/lib/deyes/
install -m 0644 "${SYSTEMD_DIR}/deyes-stereo-capture.service" /etc/systemd/system/
if [[ ! -e /etc/default/deyes-stereo-capture ]]; then
  install -D -m 0644 "${SYSTEMD_DIR}/deyes-stereo-capture.env.example" /etc/default/deyes-stereo-capture
  echo "Created /etc/default/deyes-stereo-capture; set its paths, then enable the service." >&2
fi
systemctl daemon-reload
echo "Installed. Review /etc/default/deyes-stereo-capture before: systemctl enable --now deyes-stereo-capture.service"
