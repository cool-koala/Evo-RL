#!/bin/bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
set -u

for arm_workspace in master1 master2 follow1 follow2; do
    echo "Building ${arm_workspace} with ROS2 Jazzy..."
    cd "${workspace}/${arm_workspace}"
    colcon build \
        --symlink-install \
        --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
done
