#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime="${repo_root}/third_party/cobot_magic_ros_runtime"

CONTROL_MODE="${CONTROL_MODE:-joint}"
START_CAMERAS="${START_CAMERAS:-true}"
START_CLIENT="${START_CLIENT:-false}"
SEND_ACTIONS="${SEND_ACTIONS:-false}"
LEFT_INTERFACE="${LEFT_INTERFACE:-can3}"
RIGHT_INTERFACE="${RIGHT_INTERFACE:-can1}"
HOST="${HOST:-115.190.52.37}"
JOINT_PORT="${JOINT_PORT:-5352}"
EE_PORT="${EE_PORT:-5353}"
CONDA_ENV="${CONDA_ENV:-evo-rl-ros2-jazzy}"
CONDA_SH="${CONDA_SH:-${HOME}/anaconda3/etc/profile.d/conda.sh}"

for arg in "$@"; do
  key="${arg%%:=*}"
  value="${arg#*:=}"
  if [[ "${key}" == "${arg}" ]]; then
    key="${arg%%=*}"
    value="${arg#*=}"
  fi
  case "${key}" in
    control_mode) CONTROL_MODE="${value}" ;;
    start_cameras) START_CAMERAS="${value}" ;;
    start_client) START_CLIENT="${value}" ;;
    send_actions) SEND_ACTIONS="${value}" ;;
    left_interface) LEFT_INTERFACE="${value}" ;;
    right_interface) RIGHT_INTERFACE="${value}" ;;
    host) HOST="${value}" ;;
    joint_port) JOINT_PORT="${value}" ;;
    ee_port) EE_PORT="${value}" ;;
    -h|--help|help)
      cat <<EOF
Usage:
  scripts/cobot_magic_policy.sh control_mode:=joint|ee_pose [start_cameras:=true|false] [start_client:=true|false] [send_actions:=false|true]

Examples:
  scripts/cobot_magic_policy.sh control_mode:=joint start_client:=false
  scripts/cobot_magic_policy.sh control_mode:=ee_pose start_client:=false send_actions:=false
  scripts/cobot_magic_policy.sh control_mode:=ee_pose start_client:=true send_actions:=true host:=115.190.52.37 ee_port:=5353
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: ${arg}" >&2
      exit 2
      ;;
  esac
done

activate_env() {
  if [[ -f "${CONDA_SH}" ]]; then
    # shellcheck source=/dev/null
    source "${CONDA_SH}"
    conda activate "${CONDA_ENV}"
  fi
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    set -u
  fi
  export PYTHONPATH="${repo_root}/src:${repo_root}/openpi/packages/openpi-client/src${PYTHONPATH:+:${PYTHONPATH}}"
  cd "${repo_root}"
}

start_cameras() {
  if [[ "${START_CAMERAS}" != "true" ]]; then
    return
  fi
  "${runtime}/tools/cameras.sh"
  "${runtime}/tools/wait_cameras.sh" "${CAMERA_WAIT_S:-45}"
}

activate_env

case "${CONTROL_MODE}" in
  joint)
    start_cameras
    if [[ "${SEND_ACTIONS}" == "true" ]]; then
      echo "Starting joint runtime with action execution enabled."
    else
      echo "Starting joint runtime. Client defaults can still dry-run with SEND_ACTIONS=false."
    fi
    "${repo_root}/scripts/cobot_magic_restart_runtime.sh" --mode policy --no-cameras &
    runtime_pid=$!
    ;;
  ee_pose)
    "${repo_root}/scripts/cobot_magic_restart_runtime.sh" --stop-only
    start_cameras
    echo "Starting EE Cartesian bridge on left=${LEFT_INTERFACE} right=${RIGHT_INTERFACE}."
    python -m lerobot.robots.cobot_magic_ros.cartesian_bridge \
      --left-interface "${LEFT_INTERFACE}" \
      --right-interface "${RIGHT_INTERFACE}" &
    runtime_pid=$!
    ;;
  *)
    echo "Invalid control_mode=${CONTROL_MODE}; expected joint or ee_pose." >&2
    exit 2
    ;;
esac

cleanup() {
  if [[ -n "${client_pid:-}" ]]; then
    kill "${client_pid}" 2>/dev/null || true
  fi
  if [[ -n "${runtime_pid:-}" ]]; then
    kill "${runtime_pid}" 2>/dev/null || true
    wait "${runtime_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "${START_CLIENT}" == "true" ]]; then
  echo "Starting local OpenPI client control_mode=${CONTROL_MODE} send_actions=${SEND_ACTIONS}."
  CONTROL_MODE="${CONTROL_MODE}" \
  HOST="${HOST}" \
  JOINT_PORT="${JOINT_PORT}" \
  EE_PORT="${EE_PORT}" \
  SEND_ACTIONS="${SEND_ACTIONS}" \
  RECORD="${RECORD:-false}" \
  "${repo_root}/scripts/eval_cobot_magic_cube_into_drawer_openpi.sh" dry-run &
  client_pid=$!
fi

wait "${runtime_pid}"
