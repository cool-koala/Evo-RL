#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime="${repo_root}/third_party/cobot_magic_ros_runtime"
arms_dir="${runtime}/remote_control"

mode="${COBOT_MAGIC_MODE:-policy}"
start_cameras=true
check_cameras=true
check_arms=true
stop_only=false
camera_wait_s="${CAMERA_WAIT_S:-20}"
arm_wait_s="${ARM_WAIT_S:-12}"
conda_env="${CONDA_ENV:-evo-rl-ros2-jazzy}"
remote_pid=""

usage() {
  cat <<EOF
Usage: $0 [OPTIONS]

Restart local Cobot Magic/X5 ROS2 runtime from bash, zsh, or fish.

Options:
  --mode policy|leader     policy: followers subscribe /cobot_magic/command/*
                           leader: plain leader-follower teleoperation
                           default: ${mode}
  --no-cameras             do not start/restart camera runtime
  --no-camera-check        skip waiting for camera image frames
  --no-arm-check           skip arm ROS2 topic checks
  --camera-wait-s SECONDS  camera frame wait timeout, default: ${camera_wait_s}
  --arm-wait-s SECONDS     delay before arm topic checks, default: ${arm_wait_s}
  --stop-only              stop arms, cameras, and slcand processes, then exit
  -h, --help               show this help

Examples:
  $0                       # policy/HIL/OpenPI mode, with cameras
  $0 --mode leader          # plain leader-follower teleoperation
  $0 --no-cameras           # restart arms only
  $0 --stop-only            # stop all local runtime processes
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      mode="${2:?missing value for --mode}"
      shift 2
      ;;
    --no-cameras)
      start_cameras=false
      shift
      ;;
    --no-camera-check)
      check_cameras=false
      shift
      ;;
    --no-arm-check)
      check_arms=false
      shift
      ;;
    --camera-wait-s)
      camera_wait_s="${2:?missing value for --camera-wait-s}"
      shift 2
      ;;
    --arm-wait-s)
      arm_wait_s="${2:?missing value for --arm-wait-s}"
      shift 2
      ;;
    --stop-only)
      stop_only=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${mode}" in
  policy|command)
    mode="policy"
    ;;
  leader)
    ;;
  *)
    echo "Unsupported --mode '${mode}'. Use policy or leader." >&2
    exit 2
    ;;
esac

activate_env() {
  if [[ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]]; then
    set +u
    # shellcheck source=/dev/null
    source "${HOME}/anaconda3/etc/profile.d/conda.sh"
    conda activate "${conda_env}"
    set -u
  fi
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    set -u
  fi
}

stop_runtime() {
  if [[ "${start_cameras}" == "true" || "${stop_only}" == "true" ]]; then
    echo "Stopping old Cobot Magic arm/camera/CAN runtime..."
  else
    echo "Stopping old Cobot Magic arm/CAN runtime..."
  fi
  "${arms_dir}/tools/stop_arms.sh" || true
  if [[ "${start_cameras}" == "true" || "${stop_only}" == "true" ]]; then
    "${runtime}/tools/stop_cameras.sh" || true
  fi
  sudo pkill -x slcand 2>/dev/null || true
  sudo pkill -x slcan_attach 2>/dev/null || true
}

check_arm_topics() {
  local missing=0
  local state_topics=(
    /cobot_magic/leader/joint_left
    /cobot_magic/leader/joint_right
    /cobot_magic/puppet/joint_left
    /cobot_magic/puppet/joint_right
  )
  local command_topics=(
    /cobot_magic/command/joint_left
    /cobot_magic/command/joint_right
  )

  echo
  echo "Checking arm state topics:"
  for topic in "${state_topics[@]}"; do
    printf "  %s ... " "${topic}"
    if timeout 5s ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
      echo "ok"
    else
      echo "missing"
      missing=1
    fi
  done

  if [[ "${mode}" == "policy" ]]; then
    echo
    echo "Checking policy command subscribers:"
    for topic in "${command_topics[@]}"; do
      printf "  %s ... " "${topic}"
      local info
      info="$(ros2 topic info "${topic}" 2>/dev/null || true)"
      if grep -Eq "Subscription count: [1-9][0-9]*" <<<"${info}"; then
        echo "ok"
      else
        echo "missing"
        missing=1
      fi
    done
  fi

  return "${missing}"
}

cleanup() {
  local code=$?
  if [[ -n "${remote_pid}" ]] && kill -0 "${remote_pid}" >/dev/null 2>&1; then
    echo
    echo "Stopping arm supervisor..."
    kill -INT "${remote_pid}" 2>/dev/null || true
    wait "${remote_pid}" 2>/dev/null || true
  fi
  exit "${code}"
}
trap cleanup INT TERM

activate_env

if [[ "${stop_only}" == "true" ]]; then
  stop_runtime
  echo "Stopped."
  exit 0
fi

stop_runtime

if [[ "${start_cameras}" == "true" ]]; then
  echo
  echo "Starting cameras..."
  "${runtime}/tools/camera_serial.sh" || true
  "${runtime}/tools/cameras.sh"
  if [[ "${check_cameras}" == "true" ]]; then
    "${runtime}/tools/wait_cameras.sh" "${camera_wait_s}"
  fi
fi

echo
echo "Starting arms in '${mode}' mode..."
echo "Keep this terminal open. Ctrl-C stops the arm runtime."
(
  cd "${arms_dir}"
  FOLLOWER_INPUT="${mode}" ./tools/remote.sh
) &
remote_pid="$!"

sleep "${arm_wait_s}"
if ! kill -0 "${remote_pid}" >/dev/null 2>&1; then
  echo "Arm supervisor exited early." >&2
  wait "${remote_pid}" || true
  exit 1
fi

if [[ "${check_arms}" == "true" ]]; then
  if ! check_arm_topics; then
    echo
    echo "Arm runtime check failed. Inspect the arm terminals/logs above." >&2
    kill -INT "${remote_pid}" 2>/dev/null || true
    wait "${remote_pid}" 2>/dev/null || true
    exit 1
  fi
fi

cat <<EOF

Cobot Magic runtime is ready.
Mode: ${mode}
Cameras: ${start_cameras}

Leave this terminal open while using the robot.
Press Ctrl-C here to stop arm nodes.
EOF

wait "${remote_pid}"
