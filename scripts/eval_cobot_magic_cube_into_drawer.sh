#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime="${repo_root}/third_party/cobot_magic_ros_runtime"

CONDA_ENV="${CONDA_ENV:-evo-rl-ros2-jazzy}"
CONDA_SH="${CONDA_SH:-${HOME}/anaconda3/etc/profile.d/conda.sh}"
POLICY_PATH="${POLICY_PATH:-${repo_root}/outputs/train/cobot_magic_cube_into_drawer_diffusion_success_only_v1/checkpoints/100000/pretrained_model}"
TRAIN_DATASET_REPO_ID="${TRAIN_DATASET_REPO_ID:-local/cobot_magic_cube_into_drawer_v1_success_only}"
TRAIN_DATASET_ROOT="${TRAIN_DATASET_ROOT:-${repo_root}/data/cobot_magic_cube_into_drawer_v1_success_only/lerobot}"
EVAL_ID_INPUT="${EVAL_ID:-eval_cobot_magic_cube_into_drawer_001}"
if [[ "${EVAL_ID_INPUT}" == eval_* ]]; then
  EVAL_ID="${EVAL_ID_INPUT}"
else
  EVAL_ID="eval_${EVAL_ID_INPUT}"
fi
EVAL_DATASET_REPO_ID="${EVAL_DATASET_REPO_ID:-local/${EVAL_ID}}"
EVAL_DATASET_ROOT="${EVAL_DATASET_ROOT:-${repo_root}/data/${EVAL_ID}/lerobot}"
TASK_DESC="${TASK_DESC:-cube into drawer}"
NUM_EPISODES="${NUM_EPISODES:-10}"
EPISODE_TIME_S="${EPISODE_TIME_S:-30}"
RESET_TIME_S="${RESET_TIME_S:-3}"
RESET_DURATION_S="${RESET_DURATION_S:-3.0}"
HIL_LEADER_SYNC_DURATION_S="${HIL_LEADER_SYNC_DURATION_S:-3.0}"
HIL_LEADER_RETURN_DURATION_S="${HIL_LEADER_RETURN_DURATION_S:-3.0}"
FPS="${FPS:-30}"
DEVICE="${DEVICE:-cuda}"
DISPLAY_DATA="${DISPLAY_DATA:-true}"
SEND_ACTIONS="${SEND_ACTIONS:-true}"
POLICY_NUM_INFERENCE_STEPS="${POLICY_NUM_INFERENCE_STEPS-16}"
CAMERA_WAIT_S="${CAMERA_WAIT_S:-20}"
RESET_POSE_PATH="${RESET_POSE_PATH:-${repo_root}/src/lerobot/robots/cobot_magic_ros/reset_poses/cobot_magic_ros_x5_initial_pose.json}"
SANITY_REPORT="${SANITY_REPORT:-${repo_root}/outputs/train/cobot_magic_cube_into_drawer_diffusion_success_only_v1/policy_sanity_report.json}"
SANITY_NUM_SAMPLES="${SANITY_NUM_SAMPLES:-64}"
SANITY_EXAMPLE_COUNT="${SANITY_EXAMPLE_COUNT:-5}"
SANITY_MAX_ACTION_ERROR="${SANITY_MAX_ACTION_ERROR:-1.0}"
SANITY_MAX_JOINT_TARGET_DELTA="${SANITY_MAX_JOINT_TARGET_DELTA:-0.5}"
SANITY_MAX_GRIPPER_TARGET_DELTA="${SANITY_MAX_GRIPPER_TARGET_DELTA:-1.0}"
SANITY_ACTION_RANGE_TOLERANCE="${SANITY_ACTION_RANGE_TOLERANCE:-0.05}"
EVAL_OVERWRITE="${EVAL_OVERWRITE:-false}"
EVAL_RESUME="${EVAL_RESUME:-false}"

usage() {
  cat <<EOF
Usage:
  $0 sanity   # offline checkpoint sanity check
  $0 check-arms # verify Cobot Magic arm ROS topics before HIL
  $0 hil      # run real-robot HIL evaluation and record labeled episodes
  $0 report   # summarize eval success rate

Useful overrides, valid from bash and fish:
  env EVAL_ID=cube_002 NUM_EPISODES=20 $0 hil
  env DEVICE=cpu SANITY_NUM_SAMPLES=1 $0 sanity
  env SANITY_MAX_ACTION_ERROR=1.5 $0 sanity
  env EPISODE_TIME_S=45 RESET_DURATION_S=3 RESET_TIME_S=3 $0 hil
  env HIL_LEADER_SYNC_DURATION_S=2 HIL_LEADER_RETURN_DURATION_S=2 $0 hil
  env POLICY_NUM_INFERENCE_STEPS=8 $0 hil
  env SEND_ACTIONS=false $0 hil
  env DISPLAY_DATA=false EVAL_RESUME=true $0 hil
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
  export PYTHONPATH="${repo_root}/src${PYTHONPATH:+:${PYTHONPATH}}"
  cd "${repo_root}"
}

check_arm_topics() {
  activate_env
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
  local missing=0

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

  echo "Checking Cobot Magic policy command subscribers:"
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

Arm ROS runtime is not ready for policy/HIL control.

Start it in policy command mode in a separate terminal and keep it open:

  cd ${repo_root}/third_party/cobot_magic_ros_runtime/remote_control
  source ~/anaconda3/etc/profile.d/conda.sh
  conda activate ${CONDA_ENV}
  FOLLOWER_INPUT=policy ./tools/remote.sh

Then re-run:

  $0 check-arms
  $0 hil

For plain leader-follower teleoperation or demonstration collection, use ./tools/remote.sh
without FOLLOWER_INPUT=policy. For policy/HIL evaluation, the follower nodes must subscribe
to /cobot_magic/command/joint_left and /cobot_magic/command/joint_right.
EOF
    exit 1
  fi
}

run_sanity() {
  activate_env
  python -m lerobot.scripts.check_cobot_magic_policy_sanity \
    --dataset-repo-id="${TRAIN_DATASET_REPO_ID}" \
    --dataset-root="${TRAIN_DATASET_ROOT}" \
    --policy-path="${POLICY_PATH}" \
    --device="${DEVICE}" \
    --num-samples="${SANITY_NUM_SAMPLES}" \
    --example-count="${SANITY_EXAMPLE_COUNT}" \
    --max-action-error="${SANITY_MAX_ACTION_ERROR}" \
    --max-joint-target-delta="${SANITY_MAX_JOINT_TARGET_DELTA}" \
    --max-gripper-target-delta="${SANITY_MAX_GRIPPER_TARGET_DELTA}" \
    --action-range-tolerance="${SANITY_ACTION_RANGE_TOLERANCE}" \
    --output-report="${SANITY_REPORT}"
}

run_hil() {
  check_arm_topics
  echo "Eval dataset: ${EVAL_DATASET_REPO_ID}"
  echo "Eval root: ${EVAL_DATASET_ROOT}"
  echo "Robot send_actions: ${SEND_ACTIONS}"
  echo "Episode time: ${EPISODE_TIME_S}s"
  echo "Reset duration: ${RESET_DURATION_S}s"
  echo "Reset window: ${RESET_TIME_S}s"
  echo "HIL leader sync/return: ${HIL_LEADER_SYNC_DURATION_S}s / ${HIL_LEADER_RETURN_DURATION_S}s"
  if [[ -n "${POLICY_NUM_INFERENCE_STEPS}" ]]; then
    echo "Policy num_inference_steps: ${POLICY_NUM_INFERENCE_STEPS}"
  else
    echo "Policy num_inference_steps: disabled"
  fi
  if [[ "${SEND_ACTIONS}" != "true" ]]; then
    echo "SEND_ACTIONS is not true; policy actions will not move the follower arms." >&2
    echo "Use SEND_ACTIONS=true for real HIL evaluation." >&2
    exit 1
  fi
  if [[ -x "${runtime}/tools/cameras.sh" ]]; then
    "${runtime}/tools/cameras.sh"
    "${runtime}/tools/wait_cameras.sh" "${CAMERA_WAIT_S}"
  fi

  local policy_args=(
    --policy.path="${POLICY_PATH}"
    --policy.device="${DEVICE}"
  )
  if [[ -n "${POLICY_NUM_INFERENCE_STEPS}" ]]; then
    policy_args+=(--policy.num_inference_steps="${POLICY_NUM_INFERENCE_STEPS}")
  fi

  python -m lerobot.scripts.lerobot_human_inloop_record \
    --robot.type=cobot_magic_ros_follower \
    --robot.id=cobot_magic_ros_follower_hil_eval \
    --robot.sync_gripper=true \
    --robot.send_actions="${SEND_ACTIONS}" \
    --robot.read_timeout_s=5.0 \
    --teleop.type=cobot_magic_ros_leader \
    --teleop.id=cobot_magic_ros_leader_hil_eval \
    --teleop.sync_gripper=true \
    --teleop.manual_control=true \
    --teleop.relative_takeover=true \
    --teleop.startup_sync=false \
    --reset_pose_path="${RESET_POSE_PATH}" \
    --reset_before_record=true \
    --reset_after_episode=true \
    --reset_duration_s="${RESET_DURATION_S}" \
    --hil_leader_mode=parked \
    --hil_leader_sync_duration_s="${HIL_LEADER_SYNC_DURATION_S}" \
    --hil_leader_return_duration_s="${HIL_LEADER_RETURN_DURATION_S}" \
    "${policy_args[@]}" \
    --dataset.repo_id="${EVAL_DATASET_REPO_ID}" \
    --dataset.root="${EVAL_DATASET_ROOT}" \
    --dataset.single_task="${TASK_DESC}" \
    --dataset.num_episodes="${NUM_EPISODES}" \
    --dataset.episode_time_s="${EPISODE_TIME_S}" \
    --dataset.reset_time_s="${RESET_TIME_S}" \
    --dataset.fps="${FPS}" \
    --dataset.push_to_hub=false \
    --dataset.overwrite="${EVAL_OVERWRITE}" \
    --resume="${EVAL_RESUME}" \
    --enable_episode_outcome_labeling=true \
    --require_episode_success_label=true \
    --episode_success_key=s \
    --episode_failure_key=f \
    --display_data="${DISPLAY_DATA}" \
    --play_sounds=false
}

run_report() {
  activate_env
  python -m lerobot.scripts.lerobot_dataset_report \
    --dataset="${EVAL_DATASET_REPO_ID}" \
    --root="${repo_root}/data"
}

command="${1:-}"
case "${command}" in
  sanity)
    run_sanity
    ;;
  check-arms)
    check_arm_topics
    ;;
  hil)
    run_hil
    ;;
  report)
    run_report
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
