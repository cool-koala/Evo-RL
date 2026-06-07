#!/bin/bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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

launch_terminal "follow1_right_follower" \
    "set +u && source /opt/ros/jazzy/setup.bash && source '${workspace}/follow1/install/setup.bash' && set -u && ros2 launch arm_control arx5.launch.py control_mode:=2"
sleep 1
launch_terminal "follow2_left_follower" \
    "set +u && source /opt/ros/jazzy/setup.bash && source '${workspace}/follow2/install/setup.bash' && set -u && ros2 launch arm_control arx5.launch.py control_mode:=2"
