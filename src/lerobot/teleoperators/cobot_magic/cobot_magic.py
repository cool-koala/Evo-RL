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

from lerobot.processor import RobotAction
from lerobot.robots.cobot_magic.cobot_magic import _CobotMagicArm
from lerobot.teleoperators.utils import TeleopEvents
from lerobot.utils.arx5_sdk import ARX5_GRIPPER_KEY, CobotMagicArmConfig
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_cobot_magic import CobotMagicLeaderConfig

logger = logging.getLogger(__name__)


class _CobotMagicLeaderArm(_CobotMagicArm):
    """Leader 单臂：在普通 ARX-5 适配器上增加手动示教状态切换。"""

    def __init__(self, config: CobotMagicArmConfig):
        super().__init__(config)
        self._manual_control_enabled: bool | None = None

    def set_manual_control(self, enabled: bool) -> None:
        controller = self._require_controller()
        if enabled and self._manual_control_enabled is not True:
            # damping 模式让操作者能拖动 leader；重力补偿已在 ControllerConfig 中打开。
            controller.set_to_damping()
            self._manual_control_enabled = True
            return
        if not enabled and self._manual_control_enabled is not False:
            # ARX SDK 没有单独的“退出 damping”API；下一次命令会重新进入关节控制。
            self._manual_control_enabled = False

    def get_action(self) -> RobotAction:
        # 读取真实关节反馈，确保人工拖动时 action 跟随实际 leader 姿态。
        action = self._current_action()
        if self._manual_control_enabled:
            # set_to_damping() 会把 kp 置零；持续发送当前状态让 SDK 后台循环输出阻尼和重力补偿。
            self.send_current_action(action)
        if not self.config.sync_gripper:
            # 不同步夹爪时直接省略夹爪 key，follower 会保持自己的当前夹爪位置。
            action.pop(ARX5_GRIPPER_KEY, None)
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        # 策略回放或 HIL 同步 leader 时，先退出手动示教再发送关节目标。
        self.set_manual_control(False)
        self.send_action(feedback)

    def disconnect(self) -> None:
        try:
            super().disconnect()
        finally:
            self._manual_control_enabled = None


class CobotMagicLeader(Teleoperator):
    """Cobot Magic 双臂 leader，输出可直接发送给 Cobot Magic follower 的动作。"""

    config_class = CobotMagicLeaderConfig
    name = "cobot_magic_leader"

    def __init__(self, config: CobotMagicLeaderConfig):
        super().__init__(config)
        self.config = config
        self.left_arm = _CobotMagicLeaderArm(config.left_arm_config)
        self.right_arm = _CobotMagicLeaderArm(config.right_arm_config)

    @cached_property
    def action_features(self) -> dict[str, type]:
        left_features = {f"left_{key}": value for key, value in self.left_arm.action_features.items()}
        right_features = {f"right_{key}": value for key, value in self.right_arm.action_features.items()}
        return {**left_features, **right_features}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        # feedback 使用同一套位置 key，让 policy 可以把 follower 动作同步回 leader。
        return self.action_features

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        try:
            self.left_arm.connect()
            self.right_arm.connect()
            self.configure()
        except Exception:
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
        self.set_manual_control(self.config.manual_control)

    def setup_motors(self) -> None:
        raise NotImplementedError("Cobot Magic motor setup is handled by the ARX SDK and hardware tools.")

    @check_if_not_connected
    def set_manual_control(self, enabled: bool) -> None:
        self.left_arm.set_manual_control(enabled)
        self.right_arm.set_manual_control(enabled)

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        action: RobotAction = {}
        left_action = self.left_arm.get_action()
        action.update({f"left_{key}": value for key, value in left_action.items()})
        right_action = self.right_arm.get_action()
        action.update({f"right_{key}": value for key, value in right_action.items()})
        return action

    def get_teleop_events(self) -> dict[TeleopEvents, bool]:
        # RL 处理器需要事件接口；主臂没有按钮时用配置决定是否始终人工接管。
        return {
            TeleopEvents.IS_INTERVENTION: self.config.always_intervene,
            TeleopEvents.TERMINATE_EPISODE: False,
            TeleopEvents.SUCCESS: False,
            TeleopEvents.RERECORD_EPISODE: False,
        }

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        left_feedback = {
            key.removeprefix("left_"): value for key, value in feedback.items() if key.startswith("left_")
        }
        right_feedback = {
            key.removeprefix("right_"): value for key, value in feedback.items() if key.startswith("right_")
        }
        self.left_arm.send_feedback(left_feedback)
        self.right_arm.send_feedback(right_feedback)

    def disconnect(self) -> None:
        self.left_arm.disconnect()
        self.right_arm.disconnect()
        logger.info("%s disconnected.", self)
