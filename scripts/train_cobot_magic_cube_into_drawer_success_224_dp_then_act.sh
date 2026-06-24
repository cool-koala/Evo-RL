#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

CONDA_ENV_NAME="${CONDA_ENV_NAME:-evo-rl-ros2-jazzy}"

DATASET_REPO_ID="${DATASET_REPO_ID:-local/cobot_magic_cube_into_drawer_v1_success_only_224}"
DATASET_ROOT="${DATASET_ROOT:-data/cobot_magic_cube_into_drawer_v1_success_only_224/lerobot}"

DP_OUTPUT_DIR="${DP_OUTPUT_DIR:-outputs/train/cobot_magic_cube_into_drawer_diffusion_success_only_224_v1}"
ACT_OUTPUT_DIR="${ACT_OUTPUT_DIR:-outputs/train/cobot_magic_cube_into_drawer_act_success_only_224_v1}"

DP_STEPS="${DP_STEPS:-100000}"
ACT_STEPS="${ACT_STEPS:-100000}"
DP_BATCH_SIZE="${DP_BATCH_SIZE:-16}"
ACT_BATCH_SIZE="${ACT_BATCH_SIZE:-72}"
NUM_WORKERS="${NUM_WORKERS:-4}"
SAVE_FREQ="${SAVE_FREQ:-10000}"

if command -v lerobot-train >/dev/null 2>&1; then
  TRAIN_CMD=(lerobot-train)
elif command -v conda >/dev/null 2>&1; then
  TRAIN_CMD=(conda run -n "$CONDA_ENV_NAME" lerobot-train)
else
  echo "Could not find lerobot-train or conda. Activate $CONDA_ENV_NAME first." >&2
  exit 1
fi

if [[ ! -f "$DATASET_ROOT/meta/info.json" ]]; then
  echo "Missing dataset: $DATASET_ROOT" >&2
  echo "Create it first with scripts/lerobot_resize_video_dataset.py." >&2
  exit 1
fi

mkdir -p outputs/train

echo "Training Diffusion Policy: $DP_OUTPUT_DIR"
env PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True "${TRAIN_CMD[@]}" \
  --dataset.repo_id="$DATASET_REPO_ID" \
  --dataset.root="$DATASET_ROOT" \
  --policy.type=diffusion \
  --policy.repo_id=local/cobot_magic_cube_into_drawer_diffusion_success_only_224_v1 \
  --policy.device=cuda \
  --policy.use_amp=true \
  --policy.crop_shape=null \
  --policy.horizon=16 \
  --policy.n_obs_steps=2 \
  --policy.n_action_steps=8 \
  --policy.drop_n_last_frames=7 \
  --batch_size="$DP_BATCH_SIZE" \
  --steps="$DP_STEPS" \
  --eval_freq=0 \
  --save_freq="$SAVE_FREQ" \
  --save_checkpoint=true \
  --wandb.enable=false \
  --policy.push_to_hub=false \
  --num_workers="$NUM_WORKERS" \
  --output_dir="$DP_OUTPUT_DIR" \
  2>&1 | tee "${DP_OUTPUT_DIR}.log"

echo "Training ACT: $ACT_OUTPUT_DIR"
env PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True "${TRAIN_CMD[@]}" \
  --dataset.repo_id="$DATASET_REPO_ID" \
  --dataset.root="$DATASET_ROOT" \
  --policy.type=act \
  --policy.repo_id=local/cobot_magic_cube_into_drawer_act_success_only_224_v1 \
  --policy.device=cuda \
  --policy.use_amp=true \
  --policy.chunk_size=30 \
  --policy.n_action_steps=10 \
  --batch_size="$ACT_BATCH_SIZE" \
  --steps="$ACT_STEPS" \
  --eval_freq=0 \
  --save_freq="$SAVE_FREQ" \
  --save_checkpoint=true \
  --wandb.enable=false \
  --policy.push_to_hub=false \
  --num_workers="$NUM_WORKERS" \
  --output_dir="$ACT_OUTPUT_DIR" \
  2>&1 | tee "${ACT_OUTPUT_DIR}.log"
