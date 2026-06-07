#!/bin/bash
set -euo pipefail

patterns=(
    "/remote_control/.*/install/arm_control/lib/arm_control/arm_node"
    "ros2 launch arm_control arx5.launch.py"
)

matching_pids() {
    local pattern="$1"
    pgrep -af "${pattern}" | while read -r pid command; do
        [[ -z "${pid}" ]] && continue
        [[ "${pid}" == "$$" ]] && continue
        [[ "${command}" == *"stop_arms.sh"* ]] && continue
        [[ "${command}" == *"pgrep -af"* ]] && continue
        echo "${pid}"
    done
}

found=0
for pattern in "${patterns[@]}"; do
    while read -r pid; do
        [[ -z "${pid}" ]] && continue
        found=1
        echo "Sending SIGINT to ${pid} (${pattern})"
        kill -INT "${pid}" 2>/dev/null || true
    done < <(matching_pids "${pattern}" || true)
done

if [[ "${found}" -eq 0 ]]; then
    echo "No Cobot Magic arm nodes are running."
    exit 0
fi

for _ in $(seq 1 40); do
    remaining=0
    for pattern in "${patterns[@]}"; do
        if [[ -n "$(matching_pids "${pattern}" || true)" ]]; then
            remaining=1
        fi
    done
    [[ "${remaining}" -eq 0 ]] && break
    sleep 0.25
done

for pattern in "${patterns[@]}"; do
    if [[ -n "$(matching_pids "${pattern}" || true)" ]]; then
        echo "WARNING: some processes still match ${pattern}; inspect before touching the arm."
        while read -r pid; do
            [[ -z "${pid}" ]] && continue
            ps -fp "${pid}" || true
        done < <(matching_pids "${pattern}" || true)
    fi
done
