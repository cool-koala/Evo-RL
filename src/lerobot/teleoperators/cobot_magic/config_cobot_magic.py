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

from dataclasses import dataclass

from lerobot.utils.arx5_sdk import CobotMagicArmConfig, validate_cobot_magic_arm_config

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("cobot_magic_leader")
@dataclass
class CobotMagicLeaderConfig(TeleoperatorConfig):
    """Cobot Magic 双臂 leader 配置，默认用于可拖动示教。"""

    left_arm_config: CobotMagicArmConfig
    right_arm_config: CobotMagicArmConfig
    # manual_control=true 时，连接后立即进入 ARX SDK damping 模式，并使用 SDK 重力补偿。
    manual_control: bool = True
    # Cobot Magic 主臂没有按钮事件；默认把主臂姿态视为人工接管。
    # 这样 RL smoke test 可以直接验证主从跟随。
    always_intervene: bool = True

    def __post_init__(self):
        validate_cobot_magic_arm_config(self.left_arm_config)
        validate_cobot_magic_arm_config(self.right_arm_config)
        if self.left_arm_config.interface == self.right_arm_config.interface:
            raise ValueError("Cobot Magic left and right leaders must use different interfaces.")
