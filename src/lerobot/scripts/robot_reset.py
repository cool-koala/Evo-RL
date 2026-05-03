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

"""Shared helpers for capturing and restoring robot joint reset poses."""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Any

from lerobot.processor import RobotAction
from lerobot.utils.constants import HF_LEROBOT_HOME


def default_reset_pose_path(robot_type: str, robot_id: str | None, *, namespace: str = "reset_pose") -> Path:
    robot_id = robot_id if robot_id else "default"
    return HF_LEROBOT_HOME / namespace / f"{robot_type}_{robot_id}.json"


def extract_joint_pos_from_observation(observation: dict[str, Any]) -> dict[str, float]:
    return {key: float(value) for key, value in observation.items() if key.endswith(".pos")}


def make_zero_pose(robot: Any) -> dict[str, float]:
    zero_pose = {key: 0.0 for key in robot.action_features if key.endswith(".pos")}
    if not zero_pose:
        raise ValueError("Could not build zero pose: no '.pos' joints found in robot action features.")
    return zero_pose


def _get_joint_observation(robot: Any) -> dict[str, Any]:
    get_joint_observation = getattr(robot, "get_joint_observation", None)
    if callable(get_joint_observation):
        return get_joint_observation()
    return robot.get_observation()


def capture_current_joint_pose(robot: Any) -> dict[str, float]:
    observation = _get_joint_observation(robot)
    joint_pos = extract_joint_pos_from_observation(observation)
    if not joint_pos:
        raise ValueError("Could not capture current joint pose: no '.pos' joints found in observation.")
    return joint_pos


def save_reset_pose(robot: Any, pose_path: Path) -> dict[str, float]:
    joint_pos = capture_current_joint_pose(robot)

    pose_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"robot_type": getattr(robot, "robot_type", type(robot).__name__), "joint_pos": joint_pos}
    with open(pose_path, "w") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
    logging.info("Saved reset pose to %s", pose_path)
    return joint_pos


def load_reset_pose_payload(pose_path: Path) -> dict[str, Any]:
    with open(pose_path) as f:
        payload = json.load(f)
    if not isinstance(payload, dict):
        raise ValueError(f"Invalid reset pose payload in {pose_path}: expected dict, got {type(payload)}")
    return payload


def _extract_joint_pose(payload: dict[str, Any], pose_path: Path, field_name: str) -> dict[str, float]:
    joint_pos_raw = payload.get(field_name, payload)
    if not isinstance(joint_pos_raw, dict):
        raise ValueError(
            f"Invalid reset pose payload in {pose_path}: expected {field_name!r} dict, "
            f"got {type(joint_pos_raw)}"
        )
    joint_pos = {str(key): float(value) for key, value in joint_pos_raw.items() if str(key).endswith(".pos")}
    if not joint_pos:
        raise ValueError(f"Invalid reset pose payload in {pose_path}: no '.pos' joints found.")
    return joint_pos


def load_reset_pose(pose_path: Path) -> dict[str, float]:
    payload = load_reset_pose_payload(pose_path)
    joint_pos = _extract_joint_pose(payload, pose_path, "joint_pos")
    logging.info("Loaded reset pose from %s", pose_path)
    return joint_pos


def load_optional_named_joint_pose(pose_path: Path, field_name: str) -> dict[str, float] | None:
    payload = load_reset_pose_payload(pose_path)
    if field_name not in payload:
        return None
    joint_pos = _extract_joint_pose(payload, pose_path, field_name)
    logging.info("Loaded %s from %s", field_name, pose_path)
    return joint_pos


def slow_reset_all_arms_to_pose(
    robot: Any,
    teleop: Any | None,
    target_pose: dict[str, float],
    teleop_target_pose: dict[str, float] | None = None,
    duration_s: float = 3.0,
    fps: int = 20,
) -> None:
    if duration_s <= 0:
        raise ValueError("`duration_s` must be > 0.")
    if fps <= 0:
        raise ValueError("`fps` must be > 0.")

    joint_keys = [key for key in robot.action_features if key.endswith(".pos") and key in target_pose]
    if not joint_keys:
        logging.warning("No matching '.pos' joints found for the stored reset pose.")
        return

    current_pose = extract_joint_pos_from_observation(_get_joint_observation(robot))
    start_pose = {key: current_pose.get(key, float(target_pose[key])) for key in joint_keys}
    goal_pose = {key: float(target_pose[key]) for key in joint_keys}

    if teleop is not None and not isinstance(teleop, list) and hasattr(teleop, "set_manual_control"):
        teleop.set_manual_control(False)

    teleop_joint_keys: list[str] = []
    teleop_start_pose: dict[str, float] = {}
    teleop_goal_pose: dict[str, float] = {}
    if (
        teleop_target_pose is not None
        and teleop is not None
        and not isinstance(teleop, list)
        and hasattr(teleop, "send_feedback")
    ):
        if hasattr(teleop, "get_absolute_action"):
            current_teleop_pose = teleop.get_absolute_action()
        else:
            current_teleop_pose = teleop.get_action()
        feedback_features = getattr(teleop, "feedback_features", {})
        feedback_feature_keys = list(feedback_features) if hasattr(feedback_features, "keys") else []
        candidate_keys = feedback_feature_keys or list(teleop_target_pose)
        teleop_joint_keys = [
            key
            for key in candidate_keys
            if key.endswith(".pos") and key in current_teleop_pose
        ]
        teleop_start_pose = {
            key: float(current_teleop_pose.get(key, teleop_target_pose.get(key, current_teleop_pose[key])))
            for key in teleop_joint_keys
        }
        teleop_goal_pose = {
            key: float(teleop_target_pose.get(key, current_teleop_pose[key])) for key in teleop_joint_keys
        }

    steps = max(int(duration_s * fps), 1)
    step_dt_s = duration_s / steps
    logging.info("Returning robot to reset pose over %.2fs.", duration_s)
    for idx in range(1, steps + 1):
        alpha = idx / steps
        action: RobotAction = {
            key: start_pose[key] + (goal_pose[key] - start_pose[key]) * alpha for key in joint_keys
        }
        sent_action = robot.send_action(action)
        if teleop is not None and not isinstance(teleop, list) and hasattr(teleop, "send_feedback"):
            if teleop_joint_keys:
                teleop_action: RobotAction = {
                    key: teleop_start_pose[key] + (teleop_goal_pose[key] - teleop_start_pose[key]) * alpha
                    for key in teleop_joint_keys
                }
                teleop.send_feedback(teleop_action)
            else:
                teleop.send_feedback(sent_action)
        time.sleep(step_dt_s)

    logging.info("Robot returned to the stored reset pose in %.2fs.", duration_s)
