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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.utils.arx5_sdk import CobotMagicArmConfig, validate_cobot_magic_arm_config

from ..config import RobotConfig


@RobotConfig.register_subclass("cobot_magic_follower")
@dataclass
class CobotMagicFollowerConfig(RobotConfig):
    """Cobot Magic 双臂 follower 配置，底层机械臂为两条 ARX-5/X5。"""

    # 左右臂各自对应一个 ARX SDK 控制器和一个 CAN/EtherCAT 接口。
    left_arm_config: CobotMagicArmConfig
    right_arm_config: CobotMagicArmConfig
    # 相机按 EvoRL/LeRobot 标准 CameraConfig 接入，不放进 ARX 驱动里。
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self):
        super().__post_init__()
        validate_cobot_magic_arm_config(self.left_arm_config)
        validate_cobot_magic_arm_config(self.right_arm_config)
        if self.left_arm_config.interface == self.right_arm_config.interface:
            raise ValueError("Cobot Magic left and right arms must use different interfaces.")

