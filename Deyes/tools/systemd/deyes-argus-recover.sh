#!/usr/bin/env bash
# Called by systemd with SERVICE_RESULT, EXIT_CODE and EXIT_STATUS.  This file
# must be installed root-owned and is invoked with systemd's '+' privilege
# prefix.  It never kills arbitrary processes.
set -euo pipefail

service_result="${1:-}"
exit_code="${2:-}"
exit_status="${3:-}"

# The node's documented watchdog contract is: failure/exited/75.  Do not reset
# the camera daemon for ordinary bad parameters, SIGTERM, or other faults.
if [[ "${service_result}" != "failure" || "${exit_code}" != "exited" || "${exit_status}" != "75" ]]; then
  exit 0
fi

state_dir=/run/deyes-stereo
state_file="${state_dir}/argus-recovery-epoch-seconds"
window_sec=600
max_recoveries=2
install -d -m 0755 "${state_dir}"
now="$(date +%s)"

# Keep only attempts inside the window.  A third stall is intentionally not
# repaired: systemd's StartLimit then leaves capture failed for operator review.
recent="$(awk -v now="${now}" -v window="${window_sec}" '$1 >= now - window { print $1 }' "${state_file}" 2>/dev/null || true)"
count="$(printf '%s\n' "${recent}" | sed '/^$/d' | wc -l)"
if (( count >= max_recoveries )); then
  logger -t deyes-argus-recover -- "refusing Argus restart: ${count} recoveries within ${window_sec}s; capture remains fail-closed"
  printf '%s\n' "${recent}" > "${state_file}"
  exit 0
fi

printf '%s\n%s\n' "${recent}" "${now}" | sed '/^$/d' > "${state_file}"
logger -t deyes-argus-recover -- "capture watchdog exit 75: restarting nvargus-daemon (${count}/${max_recoveries} prior attempts in ${window_sec}s)"
systemctl restart nvargus-daemon.service
systemctl is-active --quiet nvargus-daemon.service
