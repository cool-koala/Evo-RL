#!/bin/bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
    echo "Usage: $0 <master1|master2|follow1|follow2> <seconds> [ros2 launch args...]"
    exit 2
fi

arm_workspace="$1"
duration_s="$2"
shift 2

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
log_dir="${COBOT_MAGIC_DEBUG_LOG_DIR:-/tmp/cobot_magic_arm_debug}"
mkdir -p "${log_dir}"
log_file="${log_dir}/${arm_workspace}_$(date +%Y%m%d_%H%M%S).log"
node_executable="${workspace}/${arm_workspace}/install/arm_control/lib/arm_control/arm_node"

case "${arm_workspace}" in
    master1|master2|follow1|follow2) ;;
    *)
        echo "Unknown arm workspace: ${arm_workspace}"
        exit 2
        ;;
esac

cleanup() {
    local pid="${launch_pid:-}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
        echo "Stopping ${arm_workspace}; requesting motor power-off..."
        kill -INT "${pid}" 2>/dev/null || true
        for _ in $(seq 1 40); do
            if ! kill -0 "${pid}" 2>/dev/null; then
                break
            fi
            sleep 0.25
        done
        if kill -0 "${pid}" 2>/dev/null; then
            kill -TERM "${pid}" 2>/dev/null || true
            for _ in $(seq 1 20); do
                if ! kill -0 "${pid}" 2>/dev/null; then
                    break
                fi
                sleep 0.25
            done
        fi
        if kill -0 "${pid}" 2>/dev/null; then
            echo "WARNING: ${arm_workspace} did not exit after SIGINT. Check ${log_file} before touching the arm."
        fi
    fi
}
trap cleanup EXIT INT TERM

if [[ ! -x "${node_executable}" ]]; then
    echo "Missing executable: ${node_executable}"
    echo "Run ./tools/build.sh first."
    exit 1
fi

ros_args=()
for arg in "$@"; do
    if [[ "${arg}" == *":="* ]]; then
        ros_args+=("-p" "${arg}")
    else
        ros_args+=("${arg}")
    fi
done

(
    set +u
    source /opt/ros/jazzy/setup.bash
    source "${workspace}/${arm_workspace}/install/setup.bash"
    set -u
    exec "${node_executable}" --ros-args "${ros_args[@]}"
) >"${log_file}" 2>&1 &
launch_pid="$!"

echo "Started ${arm_workspace} for ${duration_s}s; log: ${log_file}"
sleep "${duration_s}"
cleanup
wait "${launch_pid}" 2>/dev/null || true
if pgrep -f "${node_executable}" >/dev/null; then
    echo "WARNING: ${arm_workspace} arm_node still appears to be running after cleanup."
    pgrep -af "${node_executable}" || true
fi
echo "Stopped ${arm_workspace}; log: ${log_file}"
