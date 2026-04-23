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

from __future__ import annotations

import logging
import time
from functools import cached_property
from typing import Any

import numpy as np

from lerobot.processor import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.dobot_xtrainer_utils import (
    ButtonPressTracker,
    JOINT_ACTION_KEYS,
    calculate_tcp_positions_and_velocities,
    check_joint_safety,
    check_pose_protection,
    clamp_joint_delta,
    convert_leader_command_to_driver_space,
    ensure_dobot_import_path,
    joint_delta_vector,
    joint_action_dict_to_array,
    joint_array_to_dict,
    load_dobot_hand_configs,
)

from ..teleoperator import Teleoperator
from .config_dobot_xtrainer_leader import DobotXTrainerLeaderConfig

LOGGER = logging.getLogger(__name__)


class DobotXTrainerLeader(Teleoperator):
    config_class = DobotXTrainerLeaderConfig
    name = "dobot_xtrainer_leader"

    def __init__(self, config: DobotXTrainerLeaderConfig):
        super().__init__(config)
        self.config = config
        self._dobot_root = ensure_dobot_import_path(config.dobot_root)
        self._is_connected = False
        self._left_agent: Any | None = None
        self._right_agent: Any | None = None
        self._manual_control_enabled = config.manual_control
        self._button_tracker = ButtonPressTracker(
            short_press_max_s=config.button_short_press_max_s,
            long_press_min_s=config.button_long_press_min_s,
            cooldown_s=config.button_cooldown_s,
        )
        self._pending_events: dict[str, Any] = {}
        self._protection_latched = False
        self._stall_counter = 0
        self._last_feedback_target: np.ndarray | None = None
        self._last_feedback_observed: np.ndarray | None = None

    @cached_property
    def action_features(self) -> dict[str, type]:
        return dict.fromkeys(JOINT_ACTION_KEYS, float)

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return dict.fromkeys(JOINT_ACTION_KEYS, float)

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        from dobot_control.agents.dobot_agent import DobotAgent

        hand_configs = load_dobot_hand_configs(self._dobot_root)
        self._left_agent = DobotAgent(
            using_sensor=self.config.leader_sensor_enabled,
            which_hand="LEFT",
            dobot_config=hand_configs["HAND_LEFT"],
        )
        self._right_agent = DobotAgent(
            using_sensor=self.config.leader_sensor_enabled,
            which_hand="RIGHT",
            dobot_config=hand_configs["HAND_RIGHT"],
        )

        self._is_connected = True
        self._protection_latched = False
        self._stall_counter = 0
        self._last_feedback_observed = self._current_joint_state()
        self._last_feedback_target = np.array(self._last_feedback_observed, copy=True)
        self.set_manual_control(self.config.manual_control)

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    def _require_agents(self) -> tuple[Any, Any]:
        if self._left_agent is None or self._right_agent is None:
            raise RuntimeError("Dobot leader is not connected.")
        return self._left_agent, self._right_agent

    def _set_torque_mode(self, enabled: bool) -> None:
        left_agent, right_agent = self._require_agents()
        left_agent.set_torque(enabled)
        right_agent.set_torque(enabled)

    def set_manual_control(self, enabled: bool) -> None:
        if not self._is_connected:
            return
        if self._protection_latched and not enabled:
            raise RuntimeError("Leader protection is latched. Run a sync step after clearing the obstacle.")
        self._set_torque_mode(not enabled)
        self._manual_control_enabled = enabled

    def _current_joint_state(self) -> np.ndarray:
        left_agent, right_agent = self._require_agents()
        left = np.asarray(left_agent.act({}), dtype=np.float64)
        right = np.asarray(right_agent.act({}), dtype=np.float64)
        return np.concatenate([left, right])

    def _current_key_state(self) -> np.ndarray:
        left_agent, right_agent = self._require_agents()
        return np.asarray([left_agent.get_keys(), right_agent.get_keys()], dtype=np.int64)

    def _sensor_triggered(self, key_state: np.ndarray | None = None) -> bool:
        if not self.config.leader_sensor_enabled:
            return False
        state = self._current_key_state() if key_state is None else np.asarray(key_state, dtype=np.int64)
        return bool(np.any(state[:, 2] > 0))

    def _clear_protection_latch(self) -> None:
        self._protection_latched = False
        self._stall_counter = 0
        current = self._current_joint_state()
        self._last_feedback_observed = current
        self._last_feedback_target = np.array(current, copy=True)

    def _trigger_protection(self, reason: str) -> None:
        LOGGER.error("Dobot leader protection triggered: %s", reason)
        self._protection_latched = True
        self._stall_counter = 0
        self._pending_events.update({"exit_early": True, "indicator_state": "red"})
        if self.config.leader_protection_auto_release_torque:
            try:
                self._set_torque_mode(False)
            finally:
                self._manual_control_enabled = True

    def _validate_target(self, target: np.ndarray, current: np.ndarray) -> np.ndarray:
        safe_target = clamp_joint_delta(target, current, self.config.leader_sync_max_joint_delta_rad)
        safe_joint, joint_warnings = check_joint_safety(safe_target)
        if not safe_joint:
            raise RuntimeError("; ".join(joint_warnings))

        safe_pose, pose_warnings = check_pose_protection(
            safe_target,
            current,
            active_arms=(True, True),
            total_time_s=1.0 / max(self.config.teleop_poll_hz, 1e-6),
            max_tcp_speed_mps=self.config.leader_sync_max_tcp_speed_mps,
        )
        if not safe_pose:
            raise RuntimeError("; ".join(pose_warnings))

        _, velocities = calculate_tcp_positions_and_velocities(
            safe_target,
            current,
            1.0 / max(self.config.teleop_poll_hz, 1e-6),
        )
        max_step = max(
            float(np.linalg.norm(velocities[name] / max(self.config.teleop_poll_hz, 1e-6)))
            for name in velocities
        )
        if max_step > self.config.leader_sync_max_cartesian_step_m:
            raise RuntimeError(
                "Leader sync cartesian step exceeded "
                f"{self.config.leader_sync_max_cartesian_step_m:.4f} m."
            )

        return safe_target

    def _send_joint_command_array(self, target: np.ndarray) -> None:
        left_agent, right_agent = self._require_agents()
        left_robot = left_agent._robot
        right_robot = right_agent._robot

        left_target = convert_leader_command_to_driver_space(left_robot, target[:7])
        right_target = convert_leader_command_to_driver_space(right_robot, target[7:])

        left_driver_target = left_target * left_robot._joint_signs + left_robot._joint_offsets
        right_driver_target = right_target * right_robot._joint_signs + right_robot._joint_offsets

        left_robot._driver.set_joints(left_driver_target.tolist())
        right_robot._driver.set_joints(right_driver_target.tolist())

    def _update_stall_state(self, current_before_send: np.ndarray) -> None:
        if self._last_feedback_target is None or self._last_feedback_observed is None:
            self._last_feedback_observed = np.array(current_before_send, copy=True)
            self._last_feedback_target = np.array(current_before_send, copy=True)
            return

        remaining = float(np.linalg.norm(joint_delta_vector(self._last_feedback_target, current_before_send)))
        progress = float(np.linalg.norm(joint_delta_vector(current_before_send, self._last_feedback_observed)))
        if remaining > self.config.leader_sync_max_joint_delta_rad and progress < self.config.leader_stall_min_progress_rad:
            self._stall_counter += 1
        else:
            self._stall_counter = 0

        self._last_feedback_observed = np.array(current_before_send, copy=True)
        if self._stall_counter >= self.config.leader_stall_max_cycles:
            raise RuntimeError("Leader motion appears stalled; possible collision or blocked axis.")

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        return joint_array_to_dict(self._current_joint_state())

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        if self._manual_control_enabled:
            return
        if self._protection_latched:
            raise RuntimeError("Leader protection is latched.")

        key_state = self._current_key_state()
        if self._sensor_triggered(key_state):
            self._trigger_protection("External leader sensor triggered.")
            raise RuntimeError("External leader sensor triggered.")

        current = self._current_joint_state()
        target = joint_action_dict_to_array(feedback, current)
        try:
            self._update_stall_state(current)
            safe_target = self._validate_target(target, current)
            self._send_joint_command_array(safe_target)
        except Exception as exc:
            self._trigger_protection(str(exc))
            raise

        self._last_feedback_target = np.array(safe_target, copy=True)

    def poll_control_events(self) -> dict[str, Any]:
        if not self.is_connected:
            return {}

        events = dict(self._pending_events)
        self._pending_events.clear()

        key_state = self._current_key_state()
        if self._sensor_triggered(key_state) and not self._protection_latched:
            self._trigger_protection("External leader sensor triggered.")
            events.update(self._pending_events)
            self._pending_events.clear()

        button_events = self._button_tracker.update(key_state)
        if button_events["toggle_intervention"]:
            events["toggle_intervention"] = True
        if button_events["sync_request"]:
            events["sync_request"] = True
        if button_events["record_toggle"]:
            events["exit_early"] = True
        return events

    def sync_to_robot(self, robot: Any) -> None:
        if self._sensor_triggered():
            raise RuntimeError("Cannot sync leader while the external safety sensor is triggered.")

        if self._protection_latched:
            self._clear_protection_latch()

        self.set_manual_control(False)
        target_dict = robot.get_feedback_action_for_teleop()
        current = self._current_joint_state()
        target = joint_action_dict_to_array(target_dict, current)

        max_delta = float(np.max(np.abs(joint_delta_vector(target, current))))
        steps = max(1, int(max_delta / max(self.config.leader_sync_max_joint_delta_rad, 1e-6)))
        steps = min(steps, 200)
        for waypoint in np.linspace(current, target, steps + 1, dtype=np.float64)[1:]:
            safe_target = self._validate_target(waypoint, self._current_joint_state())
            self._send_joint_command_array(safe_target)
            time.sleep(1.0 / max(self.config.teleop_poll_hz, 1e-6))

        self._clear_protection_latch()
        self._last_feedback_target = np.array(target, copy=True)

    def prepare_for_autonomous_start(self, robot: Any) -> None:
        if hasattr(robot, "move_to_home"):
            robot.move_to_home()
        self.sync_to_robot(robot)
        self.set_manual_control(False)
        if hasattr(robot, "set_indicator_state"):
            robot.set_indicator_state("yellow")

    @check_if_not_connected
    def disconnect(self) -> None:
        for agent in (self._left_agent, self._right_agent):
            if agent is None:
                continue
            robot = agent._robot
            try:
                robot.set_torque_mode(False)
            except Exception:
                LOGGER.debug("Failed to disable Dobot leader torque on disconnect.", exc_info=True)
            try:
                robot._driver.close()
            except Exception:
                LOGGER.debug("Failed to close Dobot leader driver on disconnect.", exc_info=True)

        self._left_agent = None
        self._right_agent = None
        self._is_connected = False
