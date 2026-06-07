#!/bin/bash
set -euo pipefail

runtime="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${runtime}/camera_ws"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "Missing ROS2 Jazzy: /opt/ros/jazzy/setup.bash"
    exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u

cd "${workspace}"
colcon build \
    --symlink-install \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
