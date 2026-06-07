#!/bin/bash
set -euo pipefail

runtime="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if pgrep -f "ros2 launch cobot_magic_cameras cobot_magic_rgb.launch.py|opencv_rgb_cameras|cobot_magic_rgb_cameras" >/dev/null 2>&1; then
    echo "Warning: camera ROS2 nodes are running. For reliable device checks, run:"
    echo "  ${runtime}/tools/stop_cameras.sh"
    echo
fi

echo "Video device symlinks by id:"
if compgen -G "/dev/v4l/by-id/*" >/dev/null; then
    ls -l /dev/v4l/by-id/*
else
    echo "  none"
fi

echo
echo "Video device symlinks by path:"
if compgen -G "/dev/v4l/by-path/*" >/dev/null; then
    ls -l /dev/v4l/by-path/*
else
    echo "  none"
fi

echo
echo "Video devices:"
if compgen -G "/dev/video*" >/dev/null; then
    ls -l /dev/video*
else
    echo "  none"
fi

if command -v v4l2-ctl >/dev/null 2>&1; then
    echo
    echo "v4l2 devices:"
    v4l2-ctl --list-devices || true
fi

echo
echo "Current camera device configuration:"
echo "  CAMERA_F_DEVICE=${CAMERA_F_DEVICE:-<auto by CAMERA_F_SERIAL>}"
echo "  CAMERA_L_DEVICE=${CAMERA_L_DEVICE:-<auto by CAMERA_L_SERIAL>}"
echo "  CAMERA_R_DEVICE=${CAMERA_R_DEVICE:-<auto by CAMERA_R_SERIAL>}"
echo "  CAMERA_F_SERIAL=${CAMERA_F_SERIAL:-AU1SB3300XB}"
echo "  CAMERA_L_SERIAL=${CAMERA_L_SERIAL:-AU1SB3300YB}"
echo "  CAMERA_R_SERIAL=${CAMERA_R_SERIAL:-AU1SB33005A}"
