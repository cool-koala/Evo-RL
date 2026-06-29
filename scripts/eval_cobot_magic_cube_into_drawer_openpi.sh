#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime="${repo_root}/third_party/cobot_magic_ros_runtime"

CONDA_ENV="${CONDA_ENV:-evo-rl-ros2-jazzy}"
CONDA_SH="${CONDA_SH:-${HOME}/anaconda3/etc/profile.d/conda.sh}"
HOST="${HOST:-115.190.52.37}"
JOINT_PORT="${JOINT_PORT:-5352}"
EE_PORT="${EE_PORT:-5353}"
CONTROL_MODE="${CONTROL_MODE:-joint}"
API_KEY="${API_KEY:-}"
EVAL_ID_INPUT="${EVAL_ID:-openpi_cube_001}"
if [[ "${EVAL_ID_INPUT}" == eval_* ]]; then
  EVAL_ID="${EVAL_ID_INPUT}"
else
  EVAL_ID="eval_${EVAL_ID_INPUT}"
fi
EVAL_DATASET_REPO_ID="${EVAL_DATASET_REPO_ID:-local/${EVAL_ID}}"
EVAL_DATASET_ROOT="${EVAL_DATASET_ROOT:-${repo_root}/data/${EVAL_ID}/lerobot}"
TASK_DESC="${TASK_DESC:-put cube in drawer}"
NUM_EPISODES="${NUM_EPISODES:-10}"
EPISODE_TIME_S="${EPISODE_TIME_S:-300}"
RESET_TIME_S="${RESET_TIME_S:-3}"
RESET_DURATION_S="${RESET_DURATION_S:-5.0}"
FPS="${FPS:-30}"
ACTION_HORIZON="${ACTION_HORIZON:-16}"
PREFETCH_REMAINING_STEPS="${PREFETCH_REMAINING_STEPS:-0}"
WARMUP_INFERENCES="${WARMUP_INFERENCES:-1}"
CONNECT_TIMEOUT_S="${CONNECT_TIMEOUT_S:-10}"
REQUEST_TIMEOUT_S="${REQUEST_TIMEOUT_S:-30}"
SEND_ACTIONS="${SEND_ACTIONS:-true}"
RECORD="${RECORD:-true}"
ENABLE_TELEOP="${ENABLE_TELEOP:-true}"
CAMERA_WAIT_S="${CAMERA_WAIT_S:-45}"
SKIP_CAMERA_RESTART="${SKIP_CAMERA_RESTART:-false}"
EVAL_OVERWRITE="${EVAL_OVERWRITE:-false}"
EVAL_RESUME="${EVAL_RESUME:-false}"
RESET_POSE_PATH="${RESET_POSE_PATH:-${repo_root}/src/lerobot/robots/cobot_magic_ros/reset_poses/cobot_magic_ros_x5_initial_pose.json}"
RESET_BEFORE_EPISODE="${RESET_BEFORE_EPISODE:-true}"
RESET_AFTER_EPISODE="${RESET_AFTER_EPISODE:-true}"
RESET_ON_EXIT="${RESET_ON_EXIT:-false}"
DEFAULT_EPISODE_SUCCESS="${DEFAULT_EPISODE_SUCCESS:-}"
REQUIRE_EPISODE_LABEL="${REQUIRE_EPISODE_LABEL:-true}"
POLICY_RELATIVE_LIMIT="${POLICY_RELATIVE_LIMIT:-false}"
ENABLE_EE_SHADOW="${ENABLE_EE_SHADOW:-false}"
SWAP_JOINT_ARMS="${SWAP_JOINT_ARMS:-true}"
INPUT_FORMAT="${INPUT_FORMAT:-aloha}"
OPENPI_STATE_FORMAT="${OPENPI_STATE_FORMAT:-observation56}"
IMAGE_SIZE="${IMAGE_SIZE:-224}"
USE_VIDEOS="${USE_VIDEOS:-true}"
VCODEC="${VCODEC:-h264}"
IMAGE_WRITER_PROCESSES="${IMAGE_WRITER_PROCESSES:-0}"
IMAGE_WRITER_THREADS="${IMAGE_WRITER_THREADS:-6}"
LOG_LEVEL="${LOG_LEVEL:-INFO}"
LOG_POLICY_STEPS="${LOG_POLICY_STEPS:-false}"

case "${CONTROL_MODE}" in
  joint|ee_pose)
    ;;
  *)
    echo "Invalid CONTROL_MODE=${CONTROL_MODE}; expected joint or ee_pose." >&2
    exit 2
    ;;
esac

usage() {
  cat <<EOF
Usage:
  $0 hil          # run OpenPI real-robot eval and record labeled episodes
  $0 check        # check ROS arm topics, cameras, and OpenPI server
  $0 dry-run      # run remote OpenPI inference without policy motion

Useful overrides:
  env EVAL_ID=openpi_cube_002 NUM_EPISODES=5 $0 hil
  env EPISODE_TIME_S=45 RESET_BEFORE_EPISODE=false $0 hil
  env SKIP_CAMERA_RESTART=true $0 hil  # reuse already-running camera nodes
  env LOG_POLICY_STEPS=true $0 hil
  env CONTROL_MODE=ee_pose JOINT_PORT=5352 EE_PORT=5353 $0 dry-run
  env SWAP_JOINT_ARMS=false $0 hil  # only if the remote server output order changes
  env SEND_ACTIONS=false $0 dry-run
  env POLICY_RELATIVE_LIMIT=true $0 hil   # re-enable robot max_relative_target clipping

OpenPI defaults:
  host=${HOST} joint_port=${JOINT_PORT} ee_port=${EE_PORT} control_mode=${CONTROL_MODE}
  image preprocessing remains the current client resize_with_pad(${IMAGE_SIZE}, ${IMAGE_SIZE}) path.
  policy_relative_limit=${POLICY_RELATIVE_LIMIT}
EOF
}

activate_env() {
  if [[ ! -f "${CONDA_SH}" ]]; then
    echo "Cannot find conda activation script: ${CONDA_SH}" >&2
    exit 1
  fi
  # shellcheck source=/dev/null
  source "${CONDA_SH}"
  conda activate "${CONDA_ENV}"
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    set -u
  fi
  export PYTHONPATH="${repo_root}/src:${repo_root}/openpi/packages/openpi-client/src${PYTHONPATH:+:${PYTHONPATH}}"
  cd "${repo_root}"
}

check_arm_topics() {
  local missing=0
  local state_topics=(
    /cobot_magic/leader/joint_left
    /cobot_magic/leader/joint_right
    /cobot_magic/puppet/joint_left
    /cobot_magic/puppet/joint_right
  )
  local command_topics
  if [[ "${CONTROL_MODE}" == "ee_pose" ]]; then
    command_topics=(
      /cobot_magic/command/ee_left
      /cobot_magic/command/ee_right
    )
  else
    command_topics=(
      /cobot_magic/command/joint_left
      /cobot_magic/command/joint_right
    )
  fi

  echo "Checking Cobot Magic arm JointState topics:"
  for topic in "${state_topics[@]}"; do
    printf "  %s ... " "${topic}"
    if timeout 5s ros2 topic echo --once "${topic}" >/dev/null 2>&1; then
      echo "ok"
    else
      echo "missing"
      missing=1
    fi
  done

  echo "Checking Cobot Magic ${CONTROL_MODE} policy command subscribers:"
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

  if [[ "${missing}" -ne 0 ]]; then
    cat >&2 <<EOF

Arm ROS runtime is not ready for OpenPI control.

Start or restart it in policy mode and keep that terminal open:
  joint:   ${repo_root}/scripts/cobot_magic_restart_runtime.sh --mode policy
  ee_pose: ${repo_root}/scripts/cobot_magic_policy.sh control_mode:=ee_pose start_client:=false send_actions:=false

Then re-run:
  $0 check
  $0 hil
EOF
    exit 1
  fi
}

check_cameras() {
  if [[ "${SKIP_CAMERA_RESTART}" == "true" ]]; then
    "${runtime}/tools/wait_cameras.sh" "${CAMERA_WAIT_S}"
    return
  fi
  if [[ -x "${runtime}/tools/cameras.sh" ]]; then
    "${runtime}/tools/cameras.sh"
    "${runtime}/tools/wait_cameras.sh" "${CAMERA_WAIT_S}"
  fi
}

check_openpi_server() {
  local port="${JOINT_PORT}"
  if [[ "${CONTROL_MODE}" == "ee_pose" ]]; then
    port="${EE_PORT}"
  fi
  python - <<PY
import sys
from pathlib import Path

repo = Path(${repo_root@Q})
sys.path.insert(0, str(repo / "openpi" / "packages" / "openpi-client" / "src"))
from openpi_client import websocket_client_policy

uri = "ws://${HOST}:${port}"
policy = websocket_client_policy.WebsocketClientPolicy(host=uri, port=None, api_key=${API_KEY@Q} or None)
print(f"OpenPI ${CONTROL_MODE} server ok: {uri}, metadata={policy.get_server_metadata()}")
try:
    policy._ws.close()
except Exception:
    pass
PY
}

common_args() {
  local args=(
    --host="${HOST}"
    --joint-port="${JOINT_PORT}"
    --ee-port="${EE_PORT}"
    --control-mode="${CONTROL_MODE}"
    --connect-timeout-s="${CONNECT_TIMEOUT_S}"
    --request-timeout-s="${REQUEST_TIMEOUT_S}"
    --fps="${FPS}"
    --num-episodes="${NUM_EPISODES}"
    --episode-time-s="${EPISODE_TIME_S}"
    --reset-time-s="${RESET_TIME_S}"
    --task="${TASK_DESC}"
    --input-format="${INPUT_FORMAT}"
    --openpi-state-format="${OPENPI_STATE_FORMAT}"
    --image-size="${IMAGE_SIZE}"
    --action-horizon="${ACTION_HORIZON}"
    --prefetch-remaining-steps="${PREFETCH_REMAINING_STEPS}"
    --warmup-inferences="${WARMUP_INFERENCES}"
    --repo-id="${EVAL_DATASET_REPO_ID}"
    --dataset-root="${EVAL_DATASET_ROOT}"
    --vcodec="${VCODEC}"
    --image-writer-processes="${IMAGE_WRITER_PROCESSES}"
    --image-writer-threads="${IMAGE_WRITER_THREADS}"
    --reset-pose-path="${RESET_POSE_PATH}"
    --reset-duration-s="${RESET_DURATION_S}"
    --log-level="${LOG_LEVEL}"
  )
  [[ "${SEND_ACTIONS}" == "true" ]] && args+=(--send-actions) || args+=(--no-send-actions)
  [[ "${RECORD}" == "true" ]] && args+=(--record) || args+=(--no-record)
  [[ "${EVAL_RESUME}" == "true" ]] && args+=(--resume) || args+=(--no-resume)
  [[ "${EVAL_OVERWRITE}" == "true" ]] && args+=(--force-overwrite) || args+=(--no-force-overwrite)
  [[ "${USE_VIDEOS}" == "true" ]] && args+=(--use-videos) || args+=(--no-use-videos)
  [[ "${ENABLE_EE_SHADOW}" == "true" ]] && args+=(--enable-ee-shadow) || args+=(--no-enable-ee-shadow)
  [[ "${SWAP_JOINT_ARMS}" == "true" ]] && args+=(--swap-joint-arms) || args+=(--no-swap-joint-arms)
  [[ "${ENABLE_TELEOP}" == "true" ]] && args+=(--enable-teleop) || args+=(--no-enable-teleop)
  [[ "${POLICY_RELATIVE_LIMIT}" == "true" ]] \
    && args+=(--policy-relative-limit) \
    || args+=(--no-policy-relative-limit)
  [[ "${RESET_BEFORE_EPISODE}" == "true" ]] \
    && args+=(--reset-before-episode) \
    || args+=(--no-reset-before-episode)
  [[ "${RESET_AFTER_EPISODE}" == "true" ]] \
    && args+=(--reset-after-episode) \
    || args+=(--no-reset-after-episode)
  [[ "${RESET_ON_EXIT}" == "true" ]] && args+=(--reset-on-exit) || args+=(--no-reset-on-exit)
  [[ "${REQUIRE_EPISODE_LABEL}" == "true" ]] \
    && args+=(--require-episode-label) \
    || args+=(--no-require-episode-label)
  [[ "${LOG_POLICY_STEPS}" == "true" ]] && args+=(--log-policy-steps) || args+=(--no-log-policy-steps)
  if [[ -n "${API_KEY}" ]]; then
    args+=(--api-key="${API_KEY}")
  fi
  if [[ -n "${DEFAULT_EPISODE_SUCCESS}" ]]; then
    args+=(--default-episode-success="${DEFAULT_EPISODE_SUCCESS}")
  fi
  printf "%s\n" "${args[@]}"
}

run_check() {
  activate_env
  check_openpi_server
  check_cameras
  check_arm_topics
}

run_hil() {
  activate_env
  check_openpi_server
  check_cameras
  check_arm_topics

  if [[ "${CONTROL_MODE}" == "ee_pose" ]]; then
    echo "OpenPI server: ${HOST}:${EE_PORT}"
  else
    echo "OpenPI server: ${HOST}:${JOINT_PORT}"
  fi
  echo "Control mode: ${CONTROL_MODE}"
  echo "Eval dataset: ${EVAL_DATASET_REPO_ID}"
  echo "Eval root: ${EVAL_DATASET_ROOT}"
  echo "Task: ${TASK_DESC}"
  echo "Send actions: ${SEND_ACTIONS}"
  echo "Policy relative limit: ${POLICY_RELATIVE_LIMIT}"
  echo "Episodes: ${NUM_EPISODES}, episode_time_s=${EPISODE_TIME_S}"
  if [[ "${SEND_ACTIONS}" != "true" ]]; then
    echo "SEND_ACTIONS is not true; policy actions will not move the follower arms." >&2
    exit 1
  fi

  mapfile -t args < <(common_args)
  python "${repo_root}/scripts/openpi_cobot_magic_hil.py" "${args[@]}"
}

run_dry_run() {
  activate_env
  check_openpi_server
  check_cameras
  check_arm_topics
  mapfile -t args < <(common_args)
  python "${repo_root}/scripts/openpi_cobot_magic_hil.py" \
    "${args[@]}" \
    --dry-run-steps="${DRY_RUN_STEPS:-30}" \
    --no-send-actions \
    --no-record
}

command="${1:-hil}"
case "${command}" in
  check)
    run_check
    ;;
  hil)
    run_hil
    ;;
  dry-run)
    run_dry_run
    ;;
  ""|-h|--help|help)
    usage
    ;;
  *)
    echo "Unknown command: ${command}" >&2
    usage >&2
    exit 2
    ;;
esac
