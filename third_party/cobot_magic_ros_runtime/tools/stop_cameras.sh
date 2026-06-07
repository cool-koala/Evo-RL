#!/bin/bash
set -euo pipefail

runtime="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pid_file="${runtime}/.run/cameras.pid"

echo "Stopping Cobot Magic ROS2 camera nodes..."

if [[ -f "${pid_file}" ]]; then
    camera_pid="$(cat "${pid_file}" || true)"
    if [[ -n "${camera_pid}" ]] && kill -0 "${camera_pid}" >/dev/null 2>&1; then
        kill "${camera_pid}" 2>/dev/null || true
        for _ in {1..20}; do
            if ! kill -0 "${camera_pid}" >/dev/null 2>&1; then
                break
            fi
            sleep 0.1
        done
        if kill -0 "${camera_pid}" >/dev/null 2>&1; then
            kill -9 "${camera_pid}" 2>/dev/null || true
        fi
    fi
    rm -f "${pid_file}"
fi

patterns=(
    "ros2 launch cobot_magic_cameras cobot_magic_rgb.launch.py"
    "/cobot_magic_cameras/lib/cobot_magic_cameras/opencv_rgb_cameras"
    "cobot_magic_cameras.opencv_rgb_cameras"
    "__node:=cobot_magic_rgb_cameras"
)

for pattern in "${patterns[@]}"; do
    (pgrep -af "${pattern}" || true) | while read -r pid command; do
        [[ -z "${pid}" ]] && continue
        [[ "${pid}" == "$$" ]] && continue
        [[ "${command}" == *"stop_cameras.sh"* ]] && continue
        [[ "${command}" == *"pgrep -af"* ]] && continue
        kill "${pid}" 2>/dev/null || true
    done
done
sleep 1

for pattern in "${patterns[@]}"; do
    (pgrep -af "${pattern}" || true) | while read -r pid command; do
        [[ -z "${pid}" ]] && continue
        [[ "${pid}" == "$$" ]] && continue
        [[ "${command}" == *"stop_cameras.sh"* ]] && continue
        [[ "${command}" == *"pgrep -af"* ]] && continue
        kill -9 "${pid}" 2>/dev/null || true
    done
done

echo "Camera nodes stopped."
