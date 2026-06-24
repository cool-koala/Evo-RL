#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export EVAL_ID="${EVAL_ID:-dp224_001}"
export POLICY_PATH="${POLICY_PATH:-${repo_root}/outputs/train/cobot_magic_cube_into_drawer_diffusion_success_only_224_v1/checkpoints/100000/pretrained_model}"
export TRAIN_DATASET_REPO_ID="${TRAIN_DATASET_REPO_ID:-local/cobot_magic_cube_into_drawer_v1_success_only_224}"
export TRAIN_DATASET_ROOT="${TRAIN_DATASET_ROOT:-${repo_root}/data/cobot_magic_cube_into_drawer_v1_success_only_224/lerobot}"
export POLICY_NUM_INFERENCE_STEPS="${POLICY_NUM_INFERENCE_STEPS:-16}"
export NUM_EPISODES="${NUM_EPISODES:-10}"

exec "${repo_root}/scripts/eval_cobot_magic_cube_into_drawer.sh" "${@:-hil}"
