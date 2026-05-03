#!/bin/bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

gnome-terminal -t "follow1_right_follower" -- bash -c "cd '${workspace}/follow1'; source devel/setup.bash && roslaunch arm_control arx5.launch control_mode:=2; exec bash;"
sleep 1
gnome-terminal -t "follow2_left_follower" -- bash -c "cd '${workspace}/follow2'; source devel/setup.bash && roslaunch arm_control arx5.launch control_mode:=2; exec bash;"
