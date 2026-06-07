#!/bin/bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

runtime="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pid_file="${runtime}/.run/cameras.pid"
log_file="${runtime}/.run/cameras.log"
timeout_s="${1:-20}"
topics=(
    "/camera_f/color/image_raw"
    "/camera_l/color/image_raw"
    "/camera_r/color/image_raw"
)

echo "Waiting for Cobot Magic camera image frames (timeout=${timeout_s}s each):"
for topic in "${topics[@]}"; do
    printf "  %s ... " "${topic}"
    if timeout "${timeout_s}" ros2 topic echo --once "${topic}" --field header >/dev/null 2>&1; then
        echo "ok"
    else
        echo "failed"
        echo "No image frame received from ${topic} within ${timeout_s}s."
        if [[ -f "${pid_file}" ]]; then
            camera_pid="$(cat "${pid_file}" || true)"
            if [[ -n "${camera_pid}" ]] && kill -0 "${camera_pid}" >/dev/null 2>&1; then
                echo "Camera launch process is still running: ${camera_pid}"
            else
                echo "Camera launch process is not running."
            fi
        fi
        if [[ -f "${log_file}" ]]; then
            echo
            echo "Camera log tail:"
            tail -n 80 "${log_file}" || true
        fi
        echo
        echo "Restart cameras with: ${runtime}/tools/cameras.sh"
        exit 1
    fi
done
