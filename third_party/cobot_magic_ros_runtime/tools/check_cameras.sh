#!/bin/bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

topics=(
    "/camera_f/color/image_raw"
    "/camera_l/color/image_raw"
    "/camera_r/color/image_raw"
)

echo "Checking Cobot Magic ROS2 RGB camera topics:"
for topic in "${topics[@]}"; do
    if ros2 topic list | grep -Fqx "${topic}"; then
        echo
        echo "${topic}: present"
        ros2 topic info "${topic}" | sed -n '1,8p'
        if timeout 5 ros2 topic echo --once "${topic}" --field header >/dev/null 2>&1; then
            echo "first frame: ok"
        else
            echo "first frame: missing"
        fi
        timeout 3 ros2 topic hz "${topic}" || true
    else
        echo
        echo "${topic}: missing"
    fi
done
