#!/bin/bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime="${repo_root}/third_party/cobot_magic_ros_runtime"

source "${HOME}/anaconda3/etc/profile.d/conda.sh"
conda activate evo-rl-ros2-jazzy

export TASK_ID="${TASK_ID:-cobot_magic_cube_into_drawer_v1}"
export TASK_DESC="${TASK_DESC:-put cube in drawer}"
export N="${N:-50}"
export EPISODE_MAX_S="${EPISODE_MAX_S:-600}"
export OVERWRITE="${OVERWRITE:-false}"
export RESUME="${RESUME:-false}"
export CAMERA_WAIT_S="${CAMERA_WAIT_S:-20}"

if [[ "${OVERWRITE}" == "true" && "${RESUME}" == "true" ]]; then
  echo "OVERWRITE=true and RESUME=true cannot be used together."
  echo "Use OVERWRITE=true to delete and re-record, or RESUME=true to append episodes."
  exit 1
fi

echo "Task id: ${TASK_ID}"
echo "Task desc: ${TASK_DESC}"
echo "Episodes: ${N}"
echo "Episode max seconds: ${EPISODE_MAX_S}"
echo "Dataset overwrite: ${OVERWRITE}"
echo "Dataset resume: ${RESUME}"

cd "${runtime}"
./tools/cameras.sh
./tools/wait_cameras.sh "${CAMERA_WAIT_S}"

cd "${repo_root}"
lerobot-record \
  --robot.type=cobot_magic_ros_follower \
  --robot.id=cobot_magic_ros_follower_record \
  --robot.sync_gripper=true \
  --robot.read_timeout_s=5.0 \
  --teleop.type=cobot_magic_ros_leader \
  --teleop.id=cobot_magic_ros_leader_record \
  --teleop.sync_gripper=true \
  --teleop.manual_control=true \
  --teleop.relative_takeover=true \
  --teleop.startup_sync=true \
  --teleop.startup_sync_duration_s=5.0 \
  --teleop.startup_sync_max_joint_delta=1.5 \
  --dataset.repo_id="local/${TASK_ID}" \
  --dataset.root="${repo_root}/data/${TASK_ID}/lerobot" \
  --dataset.single_task="${TASK_DESC}" \
  --dataset.num_episodes="${N}" \
  --dataset.episode_time_s="${EPISODE_MAX_S}" \
  --dataset.reset_time_s=8 \
  --dataset.fps=30 \
  --dataset.push_to_hub=false \
  --dataset.vcodec=h264 \
  --dataset.overwrite="${OVERWRITE}" \
  --resume="${RESUME}" \
  --intervention_state_machine_enabled=false \
  --play_sounds=false \
  --enable_episode_outcome_labeling=true \
  --require_episode_success_label=true \
  --episode_success_key=s \
  --episode_failure_key=f \
  --display_data=false
