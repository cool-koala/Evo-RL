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
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.utils import ensure_safe_goal_position
from lerobot.utils.arx5_sdk import (
    ARX5_ACTION_KEYS,
    ARX5_GRIPPER_KEY,
    ARX5_JOINT_ACTION_KEYS,
    CobotMagicArmConfig,
    arx5_state_to_action,
    arx5_state_to_observation,
    get_arx5_sdk,
    make_arx5_joint_controller,
    make_arx5_joint_state,
)
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from .config_cobot_magic import CobotMagicFollowerConfig

logger = logging.getLogger(__name__)


class _CobotMagicArm:
    """单条 ARX-5 手臂适配器；外部双臂类负责加 left_/right_ 前缀。"""

    def __init__(self, config: CobotMagicArmConfig):
        self.config = config
        self.controller: Any | None = None
        self._is_connected = False
        self._arx5 = None
        self._joint_dof = 6

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def action_features(self) -> dict[str, type]:
        # 发送给 follower 的 action 只使用位置目标；速度/力矩不进入默认 action。
        keys = ARX5_ACTION_KEYS if self.config.sync_gripper else ARX5_JOINT_ACTION_KEYS
        return dict.fromkeys(keys, float)

    @property
    def observation_features(self) -> dict[str, type]:
        # observation 保留位置、速度、力矩，便于录制数据和排查真实硬件状态。
        features: dict[str, type] = {}
        for key in ARX5_JOINT_ACTION_KEYS:
            joint_name = key.removesuffix(".pos")
            features[f"{joint_name}.pos"] = float
            features[f"{joint_name}.vel"] = float
            features[f"{joint_name}.torque"] = float
        features["gripper.pos"] = float
        features["gripper.vel"] = float
        features["gripper.torque"] = float
        return features

    def connect(self) -> None:
        # 真正连接硬件时才 import SDK，这样配置解析和无硬件单测不依赖本机安装 SDK。
        self._arx5 = get_arx5_sdk()
        self.controller = make_arx5_joint_controller(self.config)
        self._joint_dof = int(self.controller.get_robot_config().joint_dof)
        if self._joint_dof != 6:
            raise ValueError(f"Cobot Magic expects 6 arm joints, got {self._joint_dof}.")
        if self.config.reset_to_home_on_connect:
            self.controller.reset_to_home()
        self._is_connected = True

    def configure(self) -> None:
        # ARX SDK 的核心配置在 controller 创建前写入 ControllerConfig，这里保留接口对齐 Robot API。
        return

    @property
    def is_calibrated(self) -> bool:
        # ARX SDK 自带标定/限位配置；EvoRL 不在连接流程里自动写电机标定，避免误操作。
        return True

    def calibrate(self) -> None:
        logger.info(
            "Cobot Magic uses ARX SDK calibration. EvoRL does not auto-run joint calibration; "
            "use ARX SDK tools directly if hardware calibration is required."
        )

    def _require_controller(self):
        if self.controller is None or self._arx5 is None:
            raise RuntimeError("Cobot Magic arm is not connected.")
        return self.controller

    def _read_state(self):
        controller = self._require_controller()
        if not self.config.background_send_recv:
            # 非后台模式下手动收一次，避免读到长时间未刷新的状态。
            controller.recv_once()
        return controller.get_joint_state()

    def get_observation(self) -> RobotObservation:
        return arx5_state_to_observation(self._read_state())

    def _current_action(self) -> RobotAction:
        return arx5_state_to_action(self._read_state(), include_gripper=True)

    def _build_safe_action(self, action: RobotAction) -> RobotAction:
        current = self._current_action()
        target = dict(current)

        has_any_joint = any(key in action for key in ARX5_JOINT_ACTION_KEYS)
        has_all_joints = all(key in action for key in ARX5_JOINT_ACTION_KEYS)
        if has_any_joint and not has_all_joints:
            logger.warning("Ignoring partial Cobot Magic joint action; all six joint keys are required.")
        elif has_all_joints:
            for key in ARX5_JOINT_ACTION_KEYS:
                target[key] = float(action[key])

        if self.config.sync_gripper and ARX5_GRIPPER_KEY in action:
            target[ARX5_GRIPPER_KEY] = float(action[ARX5_GRIPPER_KEY])

        # 用当前状态做相对限幅，防止一次 send_action 把真实机械臂拉到很远的位置。
        safety_keys = ARX5_ACTION_KEYS if self.config.sync_gripper else ARX5_JOINT_ACTION_KEYS
        goal_present = {key: (target[key], current[key]) for key in safety_keys}
        max_relative_target = self.config.max_relative_target
        if not isinstance(max_relative_target, dict):
            max_relative_target = float(max_relative_target)
        safe_action = ensure_safe_goal_position(goal_present, max_relative_target)
        if not self.config.sync_gripper:
            # 夹爪不同步时不对外暴露 gripper action，但底层 JointState 仍要填当前位置来保持夹爪。
            safe_action[ARX5_GRIPPER_KEY] = current[ARX5_GRIPPER_KEY]
        return safe_action

    def send_action(self, action: RobotAction) -> RobotAction:
        controller = self._require_controller()
        safe_action = self._build_safe_action(action)
        cmd = make_arx5_joint_state(self._arx5, self._joint_dof, safe_action)
        controller.set_joint_cmd(cmd)
        if not self.config.background_send_recv:
            controller.send_recv_once()
        return safe_action

    def set_to_damping(self) -> None:
        if self.controller is not None:
            self.controller.set_to_damping()

    def disconnect(self) -> None:
        try:
            if self.config.set_damping_on_disconnect:
                self.set_to_damping()
        finally:
            # ARX SDK pyi 没有显式 close/disconnect；释放 Python 引用，让 SDK 自己做析构清理。
            self.controller = None
            self._arx5 = None
            self._is_connected = False


class CobotMagicFollower(Robot):
    """Cobot Magic 双臂 follower，使用两条 ARX-5 机械臂执行策略或 teleop 动作。"""

    config_class = CobotMagicFollowerConfig
    name = "cobot_magic_follower"

    def __init__(self, config: CobotMagicFollowerConfig):
        super().__init__(config)
        self.config = config
        self.left_arm = _CobotMagicArm(config.left_arm_config)
        self.right_arm = _CobotMagicArm(config.right_arm_config)
        self.cameras = make_cameras_from_configs(config.cameras)
        self._teleop_send_only_mode = False

    @property
    def _motors_ft(self) -> dict[str, type]:
        left_features = {f"left_{key}": value for key, value in self.left_arm.observation_features.items()}
        right_features = {
            f"right_{key}": value for key, value in self.right_arm.observation_features.items()
        }
        return {**left_features, **right_features}

    @property
    def _action_ft(self) -> dict[str, type]:
        left_features = {f"left_{key}": value for key, value in self.left_arm.action_features.items()}
        right_features = {f"right_{key}": value for key, value in self.right_arm.action_features.items()}
        return {**left_features, **right_features}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._action_ft

    @property
    def is_connected(self) -> bool:
        cameras_connected = self._teleop_send_only_mode or all(cam.is_connected for cam in self.cameras.values())
        return self.left_arm.is_connected and self.right_arm.is_connected and cameras_connected

    def set_teleop_send_only_mode(self, enabled: bool) -> None:
        if self.left_arm.is_connected or self.right_arm.is_connected:
            raise RuntimeError("teleop send-only mode must be configured before connecting the robot.")
        self._teleop_send_only_mode = enabled

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        connected_cameras = []
        try:
            self.left_arm.connect()
            self.right_arm.connect()
            self.configure()
            if not self._teleop_send_only_mode:
                for cam in self.cameras.values():
                    cam.connect()
                    connected_cameras.append(cam)
        except Exception:
            for cam in connected_cameras:
                cam.disconnect()
            self.left_arm.disconnect()
            self.right_arm.disconnect()
            raise
        logger.info("%s connected.", self)

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        self.left_arm.calibrate()
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def setup_motors(self) -> None:
        raise NotImplementedError("Cobot Magic motor setup is handled by the ARX SDK and hardware tools.")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        if self._teleop_send_only_mode:
            raise RuntimeError(
                f"{self} was connected in teleop send-only mode, so follower observations are unavailable."
            )

        obs: RobotObservation = {}
        left_obs = self.left_arm.get_observation()
        obs.update({f"left_{key}": value for key, value in left_obs.items()})
        right_obs = self.right_arm.get_observation()
        obs.update({f"right_{key}": value for key, value in right_obs.items()})
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.async_read()
        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        left_action = {
            key.removeprefix("left_"): value for key, value in action.items() if key.startswith("left_")
        }
        right_action = {
            key.removeprefix("right_"): value for key, value in action.items() if key.startswith("right_")
        }

        sent_left = self.left_arm.send_action(left_action)
        sent_right = self.right_arm.send_action(right_action)
        return {
            **{f"left_{key}": value for key, value in sent_left.items()},
            **{f"right_{key}": value for key, value in sent_right.items()},
        }

    def disconnect(self) -> None:
        try:
            self.left_arm.disconnect()
            self.right_arm.disconnect()
        finally:
            for cam in self.cameras.values():
                if cam.is_connected:
                    cam.disconnect()
            logger.info("%s disconnected.", self)
