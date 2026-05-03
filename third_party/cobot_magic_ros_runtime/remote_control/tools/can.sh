#!/bin/bash
set -euo pipefail

echo "Cleaning up old CAN processes..."
sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan
sudo killall slcand 2>/dev/null || true
sudo killall slcan_attach 2>/dev/null || true
sleep 1

start_can() {
    local requested_device="$1"
    local interface="$2"
    local label="$3"
    shift 3
    local device=""

    for candidate in "${requested_device}" "$@"; do
        if [[ -n "${candidate}" && -e "${candidate}" ]]; then
            device="${candidate}"
            break
        fi
    done

    if [[ -z "${device}" ]]; then
        echo "Missing ${requested_device} for ${interface} (${label})."
        echo "Check ./tools/arx_can.rules or run ./tools/arm_serial.sh."
        exit 1
    fi

    echo "Starting ${interface} from ${device} (${label})..."
    sudo slcand -c -s8 -o "${device}" "${interface}"
    sleep 0.3
    sudo ip link set "${interface}" up
}

# 机械臂 serial 顺序：右后0, 右前1, 左后2, 左前3
# 当前现场映射：can0=右后主臂, can1=右前从臂, can2=左后主臂, can3=左前从臂。
start_can /dev/canable0 can0 "Right-Rear leader" /dev/ttyACM0 /dev/ttyACM1
start_can /dev/canable1 can1 "Right-Front follower"
start_can /dev/canable2 can2 "Left-Rear leader"
start_can /dev/canable3 can3 "Left-Front follower"

echo "CAN interfaces:"
ip -details link show can0
ip -details link show can1
ip -details link show can2
ip -details link show can3
