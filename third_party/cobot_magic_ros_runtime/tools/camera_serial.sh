#!/bin/bash
set -euo pipefail

runtime="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
camera_ws="${runtime}/camera_ws"

source /opt/ros/noetic/setup.bash
source "${camera_ws}/devel/setup.bash"

if pgrep -f "camera_ws/devel/lib/astra_camera/astra_camera_node" >/dev/null 2>&1; then
    echo "Warning: camera ROS nodes are running. For reliable serial enumeration, run:"
    echo "  ${runtime}/tools/stop_cameras.sh"
    echo
fi

echo "Astra/Orbbec devices:"
if [[ -x "${camera_ws}/devel/lib/astra_camera/list_devices_node" ]]; then
    device_output="$("${camera_ws}/devel/lib/astra_camera/list_devices_node" || true)"
    echo "${device_output}"

    expected_serials=(
        "${CAMERA_F_SERIAL:-AU1SB3300XB}"
        "${CAMERA_L_SERIAL:-AU1SB3300YB}"
        "${CAMERA_R_SERIAL:-AU1SB33005A}"
    )

    echo
    echo "Expected Cobot Magic camera serials:"
    for serial in "${expected_serials[@]}"; do
        if grep -q "${serial}" <<<"${device_output}"; then
            echo "  ${serial}: present"
        else
            echo "  ${serial}: missing"
        fi
    done
else
    echo "  list_devices_node not found. Run ${runtime}/tools/build_cameras.sh first."
fi
