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

from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.dobot_xtrainer_utils import (
    CAMERA_IMAGE_SHAPE,
    CAMERA_LAYOUT,
    DOBOT_HOME_INTERMEDIATE,
    DOBOT_HOME_TARGET,
    EE_ACTION_KEYS,
    JOINT_ACTION_KEYS,
    LatestCameraFrame,
    bimanual_pose_mmdeg_to_ee_array,
    check_joint_safety,
    clamp_cartesian_step,
    clamp_joint_delta,
    ee_action_dict_to_array,
    ee_array_to_dict,
    ensure_dobot_import_path,
    is_ee_action,
    is_joint_action,
    joint_action_dict_to_array,
    joint_array_to_dict,
    load_dobot_camera_ids,
    mmdeg_pose_to_ee_array,
    move_bimanual_joints_interpolated,
    set_indicator_lights,
    quat_xyzw_to_euler_xyz_deg,
)

from ..robot import Robot
from .config_dobot_xtrainer_follower import DobotXTrainerFollowerConfig

LOGGER = logging.getLogger(__name__)


class DobotXTrainerFollower(Robot):
    config_class = DobotXTrainerFollowerConfig
    name = "dobot_xtrainer_follower"

    def __init__(self, config: DobotXTrainerFollowerConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._dobot_root = ensure_dobot_import_path(config.dobot_root)
        self._left_arm: Any | None = None
        self._right_arm: Any | None = None
        self._robot: Any | None = None
        self._camera_workers: dict[str, LatestCameraFrame] = {}
        self._camera_classes: dict[str, Any] = {}
        self._last_joint_action = np.array(DOBOT_HOME_TARGET, dtype=np.float64, copy=True)
        self._last_ee_action = np.zeros(16, dtype=np.float64)
        self._last_observation: RobotObservation | None = None
        self.cameras = {name: None for name in CAMERA_LAYOUT} if config.use_cameras else {}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        features: dict[str, type | tuple[int, int, int]] = {}
        features.update(dict.fromkeys(JOINT_ACTION_KEYS, float))
        features.update(dict.fromkeys(EE_ACTION_KEYS, float))
        if self.config.use_cameras:
            for name in CAMERA_LAYOUT:
                features[name] = CAMERA_IMAGE_SHAPE
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        keys = JOINT_ACTION_KEYS if self.config.action_mode == "joint" else EE_ACTION_KEYS
        return dict.fromkeys(keys, float)

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        from dobot_control.cameras.realsense_camera import RealSenseCamera
        from dobot_control.robots.dobot import DobotRobot
        from dobot_control.robots.robot import BimanualRobot

        self._left_arm = DobotRobot(
            robot_ip=self.config.left_robot_ip,
            no_gripper=not self.config.use_gripper,
            robot_number=2,
        )
        self._right_arm = DobotRobot(
            robot_ip=self.config.right_robot_ip,
            no_gripper=not self.config.use_gripper,
            robot_number=2,
        )
        self._robot = BimanualRobot(self._left_arm, self._right_arm)
        self._camera_classes.clear()
        self._camera_workers.clear()

        if self.config.use_cameras:
            camera_ids = load_dobot_camera_ids(self._dobot_root)
            for camera_name, spec in CAMERA_LAYOUT.items():
                camera = RealSenseCamera(device_id=camera_ids[camera_name], flip=spec["flip"])
                worker = LatestCameraFrame(camera, name=camera_name)
                worker.start()
                self._camera_classes[camera_name] = camera
                self._camera_workers[camera_name] = worker

        self._is_connected = True
        if self.config.indicator_lights_enabled:
            self.set_indicator_state("off")
        if self.config.move_to_home_on_connect:
            self.move_to_home()
        self._last_joint_action = self._current_joint_state()
        self._last_ee_action = self._current_ee_action()
        self._last_observation = self.get_observation()

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    def _require_robot(self) -> Any:
        if self._robot is None:
            raise RuntimeError("Dobot follower is not connected.")
        return self._robot

    def _current_joint_state(self) -> np.ndarray:
        robot = self._require_robot()
        return np.asarray(robot.get_joint_state(), dtype=np.float64)

    def _current_pose_state(self) -> np.ndarray:
        robot = self._require_robot()
        return np.asarray(robot.get_XYZrxryrz_state(), dtype=np.float64)

    def _current_ee_action(self) -> np.ndarray:
        joint_state = self._current_joint_state()
        pose_state = self._current_pose_state()
        return bimanual_pose_mmdeg_to_ee_array(pose_state, joint_state)

    def move_to_home(self) -> None:
        robot = self._require_robot()
        move_bimanual_joints_interpolated(
            robot,
            DOBOT_HOME_INTERMEDIATE,
            step_size_rad=self.config.joint_interp_step_rad,
            control_hz=self.config.robot_command_hz,
        )
        move_bimanual_joints_interpolated(
            robot,
            DOBOT_HOME_TARGET,
            step_size_rad=self.config.joint_interp_step_rad,
            control_hz=self.config.robot_command_hz,
        )

    def _make_joint_observation(self, joint_state: np.ndarray) -> RobotObservation:
        return joint_array_to_dict(joint_state)

    def _make_ee_observation(self, pose_state: np.ndarray, joint_state: np.ndarray) -> RobotObservation:
        return ee_array_to_dict(bimanual_pose_mmdeg_to_ee_array(pose_state, joint_state))

    def normalize_action_for_storage(
        self,
        action: RobotAction,
        *,
        source: str | None = None,
        sent_action: RobotAction | None = None,
    ) -> RobotAction:
        del source
        if self.config.action_mode == "joint":
            if is_joint_action(action):
                return joint_array_to_dict(joint_action_dict_to_array(action, self._current_joint_state()))
            if sent_action is not None and is_joint_action(sent_action):
                return joint_array_to_dict(joint_action_dict_to_array(sent_action, self._current_joint_state()))
            return self.get_feedback_action_for_teleop()

        if is_ee_action(action):
            return ee_array_to_dict(ee_action_dict_to_array(action, self._current_ee_action()))
        if sent_action is not None and is_ee_action(sent_action):
            return ee_array_to_dict(ee_action_dict_to_array(sent_action, self._current_ee_action()))
        return ee_array_to_dict(self._current_ee_action())

    def get_feedback_action_for_teleop(
        self,
        *,
        requested_action: RobotAction | None = None,
        sent_action: RobotAction | None = None,
    ) -> RobotAction:
        del requested_action, sent_action
        return joint_array_to_dict(self._current_joint_state())

    def set_indicator_state(self, state: str) -> None:
        if not self.config.indicator_lights_enabled or self._robot is None:
            return
        set_indicator_lights(self._robot, state)

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        joint_state = self._current_joint_state()
        pose_state = self._current_pose_state()
        observation: RobotObservation = {}
        observation.update(self._make_joint_observation(joint_state))
        observation.update(self._make_ee_observation(pose_state, joint_state))
        for camera_name, worker in self._camera_workers.items():
            observation[camera_name] = worker.get()
        self._last_observation = observation
        return observation

    def _infer_action_mode(self, action: RobotAction) -> str:
        has_joint = is_joint_action(action)
        has_ee = is_ee_action(action)
        if has_joint and not has_ee:
            return "joint"
        if has_ee and not has_joint:
            return "ee_pose"
        return self.config.action_mode

    def _send_joint_action(self, action: RobotAction) -> RobotAction:
        robot = self._require_robot()
        current = self._current_joint_state()
        target = joint_action_dict_to_array(action, current)
        safe_target = clamp_joint_delta(target, current, self.config.joint_delta_limit_rad)
        safe, warnings = check_joint_safety(safe_target)
        if not safe:
            raise RuntimeError("; ".join(warnings))
        robot.command_joint_state(safe_target, np.array([1, 1], dtype=np.int64))
        self._last_joint_action = safe_target
        return joint_array_to_dict(safe_target)

    def _send_ee_action(self, action: RobotAction) -> RobotAction:
        if self._left_arm is None or self._right_arm is None:
            raise RuntimeError("Dobot follower is not connected.")

        current = self._current_ee_action()
        target = ee_action_dict_to_array(action, current)
        target[:8] = clamp_cartesian_step(
            target[:8],
            current[:8],
            max_translation_m=self.config.ee_translation_limit_m,
            max_rotation_deg=self.config.ee_rotation_limit_deg,
        )
        target[8:] = clamp_cartesian_step(
            target[8:],
            current[8:],
            max_translation_m=self.config.ee_translation_limit_m,
            max_rotation_deg=self.config.ee_rotation_limit_deg,
        )

        left_euler = quat_xyzw_to_euler_xyz_deg(target[3:7])
        right_euler = quat_xyzw_to_euler_xyz_deg(target[11:15])
        self._left_arm.robot.MovL(*(target[:3] * 1000.0), *left_euler)
        self._right_arm.robot.MovL(*(target[8:11] * 1000.0), *right_euler)

        if getattr(self._left_arm, "_use_gripper", False):
            self._left_arm.gripper.move(int(np.clip(target[7], 0.0, 1.0) * 255), 100, 1)
        if getattr(self._right_arm, "_use_gripper", False):
            self._right_arm.gripper.move(int(np.clip(target[15], 0.0, 1.0) * 255), 100, 1)

        if self.config.ee_command_settle_s > 0:
            time.sleep(self.config.ee_command_settle_s)
        self._last_ee_action = target
        return ee_array_to_dict(target)

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        mode = self._infer_action_mode(action)
        if mode == "joint":
            return self._send_joint_action(action)
        if mode == "ee_pose":
            return self._send_ee_action(action)
        raise ValueError(f"Unsupported Dobot Xtrainer action mode: {mode}")

    @check_if_not_connected
    def disconnect(self) -> None:
        for worker in self._camera_workers.values():
            worker.stop()
        self._camera_workers.clear()

        for arm in (self._left_arm, self._right_arm):
            if arm is None:
                continue
            stop_event = getattr(arm, "_stop_thread", None)
            thread = getattr(arm, "_reading_thread", None)
            if stop_event is not None:
                stop_event.set()
            if thread is not None:
                thread.join(timeout=1.0)
            dashboard = getattr(arm, "r_inter", None)
            if dashboard is not None and hasattr(dashboard, "DisableRobot"):
                try:
                    dashboard.DisableRobot()
                except Exception:
                    LOGGER.debug("Failed to disable Dobot follower on disconnect.", exc_info=True)

        self._left_arm = None
        self._right_arm = None
        self._robot = None
        self._is_connected = False
