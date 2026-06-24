#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Offline sanity-check a Cobot Magic policy before running it on hardware."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.configs.types import FeatureType
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.policies.utils import make_robot_action
from lerobot.utils.constants import ACTION, OBS_STATE
from lerobot.utils.control_utils import predict_action

DEFAULT_DATASET_ROOT = Path("data/cobot_magic_open_drawer_local_bimanual_v1/lerobot")
DEFAULT_REPO_ID = "local/cobot_magic_open_drawer_local_bimanual_v1"


def image_to_inference_array(image: Any) -> np.ndarray:
    if isinstance(image, torch.Tensor):
        if image.ndim == 3 and image.shape[0] in {1, 3, 4}:
            image = image[:3].permute(1, 2, 0)
        image_np = image.detach().cpu().numpy()
    else:
        image_np = np.asarray(image)
    if np.issubdtype(image_np.dtype, np.floating):
        image_np = np.clip(image_np, 0.0, 1.0) * 255.0
    return image_np.astype(np.uint8)


def policy_visual_feature_keys(cfg: PreTrainedConfig) -> list[str]:
    return [
        key
        for key, feature in cfg.input_features.items()
        if getattr(feature, "type", None) is FeatureType.VISUAL
    ]


def item_to_inference_observation(item: dict[str, Any], image_keys: list[str]) -> dict[str, np.ndarray]:
    state = item[OBS_STATE]
    state_np = state.detach().cpu().numpy() if isinstance(state, torch.Tensor) else np.asarray(state)

    observation = {OBS_STATE: state_np.astype(np.float32)}
    for key in image_keys:
        observation[key] = image_to_inference_array(item[key])
    return observation


def state_position_indices_for_actions(action_names: list[str], state_names: list[str]) -> list[int]:
    missing = [name for name in action_names if name not in state_names]
    if missing:
        raise ValueError(f"State feature is missing action position names: {missing}")
    return [state_names.index(name) for name in action_names]


def evenly_spaced_indices(length: int, num_samples: int) -> list[int]:
    if length <= 0:
        raise ValueError("Dataset is empty.")
    if num_samples <= 0 or num_samples >= length:
        return list(range(length))
    return np.linspace(0, length - 1, num=num_samples, dtype=np.int64).tolist()


def summarize(values: list[float]) -> dict[str, float]:
    arr = np.asarray(values, dtype=np.float64)
    return {
        "max": float(np.max(arr)),
        "mean": float(np.mean(arr)),
        "p50": float(np.quantile(arr, 0.50)),
        "p90": float(np.quantile(arr, 0.90)),
        "p99": float(np.quantile(arr, 0.99)),
    }


def max_abs_subset(values: np.ndarray, indices: list[int]) -> float:
    if not indices:
        return 0.0
    return float(np.max(values[indices]))


def run_check(args: argparse.Namespace) -> dict[str, Any]:
    dataset = LeRobotDataset(args.dataset_repo_id, root=args.dataset_root, video_backend=args.video_backend)
    cfg = PreTrainedConfig.from_pretrained(str(args.policy_path), cli_overrides=[f"--device={args.device}"])
    # `config.json` stores `pretrained_path=null` for local training checkpoints. Set it from the CLI
    # path so `make_policy` loads the saved weights instead of instantiating a fresh policy.
    cfg.pretrained_path = args.policy_path
    policy = make_policy(cfg, ds_meta=dataset.meta)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=cfg,
        pretrained_path=cfg.pretrained_path,
        dataset_stats=dataset.meta.stats,
        preprocessor_overrides={"device_processor": {"device": cfg.device}},
    )
    image_keys = policy_visual_feature_keys(cfg)
    missing_image_keys = [key for key in image_keys if key not in dataset.features]
    if missing_image_keys:
        raise ValueError(f"Dataset is missing policy visual input keys: {missing_image_keys}")

    action_names = list(dataset.features[ACTION]["names"])
    state_names = list(dataset.features[OBS_STATE]["names"])
    state_pos_indices = state_position_indices_for_actions(action_names, state_names)
    action_stats = dataset.meta.stats[ACTION]
    action_min = np.asarray(action_stats["min"], dtype=np.float32)
    action_max = np.asarray(action_stats["max"], dtype=np.float32)
    gripper_indices = [idx for idx, name in enumerate(action_names) if "gripper" in name]
    joint_indices = [idx for idx in range(len(action_names)) if idx not in gripper_indices]

    indices = evenly_spaced_indices(len(dataset), args.num_samples)
    max_action_errors: list[float] = []
    max_target_deltas_all: list[float] = []
    max_target_deltas_joints: list[float] = []
    max_target_deltas_grippers: list[float] = []
    out_of_range_count = 0
    examples: list[dict[str, Any]] = []

    for index in indices:
        item = dataset[index]
        policy.reset()
        preprocessor.reset()
        postprocessor.reset()
        observation = item_to_inference_observation(item, image_keys)
        predicted_action_tensor = predict_action(
            observation,
            policy,
            torch.device(cfg.device),
            preprocessor,
            postprocessor,
            cfg.use_amp,
            task=item.get("task", args.task),
            robot_type=dataset.meta.robot_type,
        )
        predicted_action = make_robot_action(predicted_action_tensor, dataset.features)
        pred = np.asarray([predicted_action[name] for name in action_names], dtype=np.float32)
        gt = (
            item[ACTION].detach().cpu().numpy()
            if isinstance(item[ACTION], torch.Tensor)
            else np.asarray(item[ACTION])
        )
        state = (
            item[OBS_STATE].detach().cpu().numpy()
            if isinstance(item[OBS_STATE], torch.Tensor)
            else np.asarray(item[OBS_STATE])
        )
        state_pos = state[state_pos_indices]

        max_action_error = float(np.max(np.abs(pred - gt)))
        target_delta = np.abs(pred - state_pos)
        max_target_delta = float(np.max(target_delta))
        max_target_delta_joints = max_abs_subset(target_delta, joint_indices)
        max_target_delta_grippers = max_abs_subset(target_delta, gripper_indices)
        max_action_errors.append(max_action_error)
        max_target_deltas_all.append(max_target_delta)
        max_target_deltas_joints.append(max_target_delta_joints)
        max_target_deltas_grippers.append(max_target_delta_grippers)
        if np.any(pred < action_min - args.action_range_tolerance) or np.any(
            pred > action_max + args.action_range_tolerance
        ):
            out_of_range_count += 1

        if len(examples) < args.example_count:
            examples.append(
                {
                    "index": int(index),
                    "episode_index": int(item["episode_index"]),
                    "frame_index": int(item["frame_index"]),
                    "max_abs_pred_minus_action": max_action_error,
                    "max_abs_pred_minus_state_pos": max_target_delta,
                    "max_abs_joint_pred_minus_state_pos": max_target_delta_joints,
                    "max_abs_gripper_pred_minus_state_pos": max_target_delta_grippers,
                    "pred_action_first_7": pred[:7].astype(float).tolist(),
                    "dataset_action_first_7": gt[:7].astype(float).tolist(),
                }
            )

    report = {
        "dataset_repo_id": args.dataset_repo_id,
        "dataset_root": str(args.dataset_root),
        "policy_path": str(args.policy_path),
        "policy_type": cfg.type,
        "visual_feature_keys": image_keys,
        "num_samples": len(indices),
        "max_abs_pred_minus_action": summarize(max_action_errors),
        "max_abs_pred_minus_state_pos": summarize(max_target_deltas_all),
        "max_abs_joint_pred_minus_state_pos": summarize(max_target_deltas_joints),
        "max_abs_gripper_pred_minus_state_pos": summarize(max_target_deltas_grippers),
        "out_of_action_range_count": out_of_range_count,
        "thresholds": {
            "max_action_error": args.max_action_error,
            "max_joint_target_delta": args.max_joint_target_delta,
            "max_gripper_target_delta": args.max_gripper_target_delta,
            "action_range_tolerance": args.action_range_tolerance,
        },
        "passed": (
            max(max_action_errors) <= args.max_action_error
            and max(max_target_deltas_joints) <= args.max_joint_target_delta
            and max(max_target_deltas_grippers) <= args.max_gripper_target_delta
            and out_of_range_count == 0
        ),
        "examples": examples,
    }
    if args.output_report is not None:
        args.output_report.parent.mkdir(parents=True, exist_ok=True)
        args.output_report.write_text(json.dumps(report, indent=2, sort_keys=True))
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-repo-id", default=DEFAULT_REPO_ID)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--policy-path", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--video-backend", default="torchcodec")
    parser.add_argument("--task", default="open the drawer")
    parser.add_argument("--num-samples", type=int, default=64)
    parser.add_argument("--example-count", type=int, default=5)
    parser.add_argument("--max-action-error", type=float, default=0.5)
    parser.add_argument("--max-joint-target-delta", type=float, default=0.5)
    parser.add_argument("--max-gripper-target-delta", type=float, default=1.0)
    parser.add_argument("--action-range-tolerance", type=float, default=0.05)
    parser.add_argument("--output-report", type=Path)
    return parser


def main() -> None:
    report = run_check(build_parser().parse_args())
    print(json.dumps(report, indent=2, sort_keys=True))
    if not report["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
