#!/bin/bash
set -euo pipefail

rules_file="/etc/udev/rules.d/arx_can.rules"

if [[ -f "${rules_file}" ]]; then
    echo "udev rules installed: ${rules_file}"
else
    echo "udev rules not installed. Run ./tools/set.sh from remote_control first."
fi

echo
echo "Current CANable serials:"
for dev in /dev/ttyACM*; do
    [[ -e "${dev}" ]] || continue
    echo "${dev}"
    udevadm info -a -n "${dev}" \
        | grep -E 'ATTRS\{serial\}|ATTRS\{idVendor\}|ATTRS\{idProduct\}' \
        | head -n 6 \
        || true
    echo
done

echo "Expected symlinks:"
ls -l /dev/canable* 2>/dev/null || true
