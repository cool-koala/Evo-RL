#!/bin/bash
set -euo pipefail

runtime="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
camera_ws="${runtime}/camera_ws"

if [[ ! -f "${camera_ws}/devel/setup.bash" ]]; then
    echo "Camera workspace is not built: ${camera_ws}/devel/setup.bash"
    echo "Run: ${runtime}/tools/build_cameras.sh"
    exit 1
fi

launch_terminal() {
    local title="$1"
    local command="$2"

    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal -t "${title}" -- bash -lc "${command}; exec bash"
    else
        echo "gnome-terminal not found; starting ${title} in background."
        bash -lc "${command}" &
    fi
}

ros_setup="source /opt/ros/noetic/setup.bash && source '${camera_ws}/devel/setup.bash'"
camera_f_serial="${CAMERA_F_SERIAL:-AU1SB3300XB}"
camera_l_serial="${CAMERA_L_SERIAL:-AU1SB3300YB}"
camera_r_serial="${CAMERA_R_SERIAL:-AU1SB33005A}"

if pgrep -f "camera_ws/devel/lib/astra_camera/astra_camera_node|roslaunch astra_camera cobot_magic_rgb.launch" >/dev/null 2>&1; then
    "${runtime}/tools/stop_cameras.sh"
fi

source /opt/ros/noetic/setup.bash
source "${camera_ws}/devel/setup.bash"
device_output="$("${camera_ws}/devel/lib/astra_camera/list_devices_node" || true)"
missing_serials=()
for serial in "${camera_f_serial}" "${camera_l_serial}" "${camera_r_serial}"; do
    if ! grep -q "${serial}" <<<"${device_output}"; then
        missing_serials+=("${serial}")
    fi
done

if (( ${#missing_serials[@]} > 0 )); then
    echo "Missing Astra/Orbbec camera serial(s): ${missing_serials[*]}"
    echo
    echo "${device_output}"
    echo
    echo "Do not start Cobot Magic cameras until all three cameras are visible."
    echo "Check USB cable, hub power, and re-plug the missing camera, then run:"
    echo "  ${runtime}/tools/camera_serial.sh"
    exit 1
fi

echo "Starting three Astra/Orbbec RGB cameras."
echo "  camera_f: ${camera_f_serial} -> /camera_f/color/image_raw"
echo "  camera_l: ${camera_l_serial} -> /camera_l/color/image_raw"
echo "  camera_r: ${camera_r_serial} -> /camera_r/color/image_raw"

launch_terminal "cobot_magic_cameras_astra" \
    "${ros_setup} && roslaunch astra_camera cobot_magic_rgb.launch \
        camera_f_serial:='${camera_f_serial}' \
        camera_l_serial:='${camera_l_serial}' \
        camera_r_serial:='${camera_r_serial}'"

echo
echo "Check camera streams with:"
echo "  ${runtime}/tools/check_cameras.sh"
