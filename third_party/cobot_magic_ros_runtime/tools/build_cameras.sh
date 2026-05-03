#!/bin/bash
set -euo pipefail

runtime="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${runtime}/camera_ws"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    echo "Missing ROS Noetic: /opt/ros/noetic/setup.bash"
    exit 1
fi

source /opt/ros/noetic/setup.bash
cd "${workspace}"
catkin_make \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
