#!/bin/bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

topics=(
    "/camera_f/color/image_raw"
    "/camera_l/color/image_raw"
    "/camera_r/color/image_raw"
)

echo "Checking Cobot Magic Astra/Orbbec camera topics:"
for topic in "${topics[@]}"; do
    if rostopic list | grep -qx "${topic}"; then
        echo
        echo "${topic}: present"
        rostopic info "${topic}" | sed -n '1,8p'
        timeout 3 rostopic hz "${topic}" || true
    else
        echo
        echo "${topic}: missing"
    fi
done
