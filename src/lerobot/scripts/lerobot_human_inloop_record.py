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

"""
Records with policy execution and teleop-device action mirroring enabled.

Phase A/B goals:
- Same policy action is executed on the follower robot and mirrored to the teleop arm.
- Keyboard-toggled intervention lets teleop temporarily take over execution.
"""

import logging
from pathlib import Path
from typing import Any

from lerobot.configs import parser
from lerobot.scripts.lerobot_record import RecordConfig, record
from lerobot.scripts.recording_hil import HIL_LEADER_MODE_PIPER
from lerobot.scripts.robot_reset import (
    default_reset_pose_path,
    extract_joint_pos_from_observation,
    load_reset_pose,
    save_reset_pose,
    slow_reset_all_arms_to_pose,
)
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.recording_annotations import (
    EPISODE_FAILURE,
    EPISODE_SUCCESS,
    infer_collector_policy_version,
)


def _default_failure_reset_pose_path(cfg: RecordConfig) -> Path:
    robot_id = cfg.robot.id if cfg.robot.id else "default"
    robot_type = cfg.robot.type if hasattr(cfg.robot, "type") else type(cfg.robot).__name__
    return default_reset_pose_path(robot_type, robot_id, namespace="failure_reset_pose")


def _extract_joint_pos_from_observation(observation: dict[str, Any]) -> dict[str, float]:
    return extract_joint_pos_from_observation(observation)


def _save_failure_reset_pose(robot: Any, pose_path: Path) -> dict[str, float]:
    return save_reset_pose(robot=robot, pose_path=pose_path)


def _load_failure_reset_pose(pose_path: Path) -> dict[str, float]:
    return load_reset_pose(pose_path)


def _slow_reset_all_arms_to_pose(
    robot: Any,
    teleop: Any,
    target_pose: dict[str, float],
    duration_s: float = 3.0,
) -> None:
    slow_reset_all_arms_to_pose(
        robot=robot,
        teleop=teleop,
        target_pose=target_pose,
        duration_s=duration_s,
        fps=20,
    )


class _HumanInloopFailureResetController:
    def __init__(self, cfg: RecordConfig):
        self.pose_path = _default_failure_reset_pose_path(cfg)
        self.failure_reset_pose: dict[str, float] | None = None

    def on_record_connected(self, robot: Any, teleop: Any) -> None:
        if self.pose_path.is_file():
            self.failure_reset_pose = _load_failure_reset_pose(self.pose_path)
            return

        input(
            "Human-inloop with policy detected.\n"
            "Please ensure ALL robot arms are at reset position, then press ENTER to capture:\n"
            f"{self.pose_path}\n"
        )
        self.failure_reset_pose = _save_failure_reset_pose(robot=robot, pose_path=self.pose_path)

    def on_episode_outcome(self, robot: Any, teleop: Any, episode_success: str | None) -> None:
        if episode_success in {EPISODE_FAILURE, EPISODE_SUCCESS} and self.failure_reset_pose is not None:
            _slow_reset_all_arms_to_pose(robot=robot, teleop=teleop, target_pose=self.failure_reset_pose)


@parser.wrap()
def human_inloop_record(cfg: RecordConfig):
    if cfg.teleop is None:
        raise ValueError("`lerobot-human-inloop-record` requires `teleop` config.")

    cfg.policy_sync_to_teleop = cfg.policy is not None and cfg.hil_leader_mode == HIL_LEADER_MODE_PIPER
    cfg.intervention_state_machine_enabled = cfg.policy is not None
    cfg.enable_episode_outcome_labeling = True
    cfg.default_episode_success = "failure"
    cfg.enable_collector_policy_id = True
    if cfg.collector_policy_id_policy is None:
        cfg.collector_policy_id_policy = infer_collector_policy_version(cfg.policy)
    if cfg.policy is not None and not cfg.reset_after_episode:
        failure_reset_controller = _HumanInloopFailureResetController(cfg)
        cfg._on_record_connected = failure_reset_controller.on_record_connected
        cfg._on_record_episode_outcome = failure_reset_controller.on_episode_outcome
    elif cfg.policy is not None:
        logging.info(
            "Skipping human-in-loop outcome reset because `reset_after_episode=true`; "
            "`lerobot_record` will run the configured reset once after each labeled episode."
        )

    logging.info(
        "Human-in-loop recording is enabled. Press '%s' to toggle takeover. "
        "Press '%s' to mark success and end, '%s' to mark failure and end. "
        "Recorded `action` is the executed action. "
        "Policy output (when policy is enabled) is stored in `complementary_info.policy_action`. "
        "Collector source is stored in `complementary_info.collector_policy_id`. "
        "ACP inference: enable=%s use_cfg=%s cfg_beta=%.3f.",
        cfg.intervention_toggle_key,
        cfg.episode_success_key,
        cfg.episode_failure_key,
        cfg.acp_inference.enable,
        cfg.acp_inference.use_cfg,
        cfg.acp_inference.cfg_beta,
    )
    return record(cfg)


def main():
    register_third_party_plugins()
    human_inloop_record()


if __name__ == "__main__":
    main()
