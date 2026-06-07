#!/bin/bash
set -euo pipefail

runtime="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
camera_ws="${runtime}/camera_ws"
state_dir="${runtime}/.run"
pid_file="${state_dir}/cameras.pid"
log_file="${state_dir}/cameras.log"

if [[ ! -f "${camera_ws}/install/setup.bash" ]]; then
    echo "Camera workspace is not built: ${camera_ws}/install/setup.bash"
    echo "Run: ${runtime}/tools/build_cameras.sh"
    exit 1
fi

ros_setup="set +u && source /opt/ros/jazzy/setup.bash && source '${camera_ws}/install/setup.bash' && set -u"
camera_f_device="${CAMERA_F_DEVICE:-}"
camera_l_device="${CAMERA_L_DEVICE:-}"
camera_r_device="${CAMERA_R_DEVICE:-}"
camera_f_serial="${CAMERA_F_SERIAL:-AU1SB3300XB}"
camera_l_serial="${CAMERA_L_SERIAL:-AU1SB3300YB}"
camera_r_serial="${CAMERA_R_SERIAL:-AU1SB33005A}"
camera_fourcc="${CAMERA_FOURCC:-MJPG}"

"${runtime}/tools/stop_cameras.sh"

missing_cameras=()
resolved_device=""
resolve_device() {
    local label="$1"
    local device="$2"
    local serial="$3"

    resolved_device=""
    if [[ -z "${device}" ]]; then
        local candidate
        for candidate in /dev/v4l/by-id/*"${serial}"* /dev/v4l/by-path/*"${serial}"*; do
            if [[ -e "${candidate}" ]]; then
                resolved_device="${candidate}"
                return
            fi
        done
        missing_cameras+=("${label}:serial=${serial}")
        return
    fi
    if [[ "${device}" =~ ^[0-9]+$ ]]; then
        device="/dev/video${device}"
    fi
    if [[ ! -e "${device}" ]]; then
        missing_cameras+=("${label}:${device}")
    fi
    resolved_device="${device}"
}

resolve_device "camera_f" "${camera_f_device}" "${camera_f_serial}"
camera_f_resolved="${resolved_device}"
resolve_device "camera_l" "${camera_l_device}" "${camera_l_serial}"
camera_l_resolved="${resolved_device}"
resolve_device "camera_r" "${camera_r_device}" "${camera_r_serial}"
camera_r_resolved="${resolved_device}"

if (( ${#missing_cameras[@]} > 0 )); then
    echo "Missing configured camera(s): ${missing_cameras[*]}"
    echo "List local video devices with: ${runtime}/tools/camera_serial.sh"
    echo "Override devices with CAMERA_F_DEVICE, CAMERA_L_DEVICE, and CAMERA_R_DEVICE."
    exit 1
fi

echo "Starting three ROS2 OpenCV RGB cameras."
echo "  fourcc: ${camera_fourcc}"
echo "  camera_f: ${camera_f_resolved} -> /camera_f/color/image_raw"
echo "  camera_l: ${camera_l_resolved} -> /camera_l/color/image_raw"
echo "  camera_r: ${camera_r_resolved} -> /camera_r/color/image_raw"

mkdir -p "${state_dir}"
launch_command="${ros_setup} && exec ros2 launch cobot_magic_cameras cobot_magic_rgb.launch.py \
        camera_f_device:='${camera_f_resolved}' \
        camera_l_device:='${camera_l_resolved}' \
        camera_r_device:='${camera_r_resolved}' \
        fourcc:='${camera_fourcc}' \
        camera_f_serial:='${camera_f_serial}' \
        camera_l_serial:='${camera_l_serial}' \
        camera_r_serial:='${camera_r_serial}'"

nohup bash -lc "${launch_command}" >"${log_file}" 2>&1 &
camera_pid="$!"
echo "${camera_pid}" > "${pid_file}"

sleep 1
if ! kill -0 "${camera_pid}" >/dev/null 2>&1; then
    echo "Camera launch exited early. Log:"
    tail -n 80 "${log_file}" || true
    exit 1
fi

echo "Camera launch PID: ${camera_pid}"
echo "Camera log: ${log_file}"

echo
echo "Check camera streams with:"
echo "  ${runtime}/tools/check_cameras.sh"
