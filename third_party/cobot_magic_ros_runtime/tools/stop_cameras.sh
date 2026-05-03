#!/bin/bash
set -euo pipefail

echo "Stopping Cobot Magic Astra/Orbbec camera nodes..."
pkill -f "roslaunch astra_camera cobot_magic_rgb.launch" 2>/dev/null || true
pkill -f "camera_ws/devel/lib/astra_camera/astra_camera_node" 2>/dev/null || true
sleep 1
echo "Camera nodes stopped."
