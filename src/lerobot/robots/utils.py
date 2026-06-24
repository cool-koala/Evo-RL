# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

from typing import cast

from lerobot.utils.import_utils import make_device_from_device_class

from .config import RobotConfig
from .robot import Robot


def make_robot_from_config(config: RobotConfig) -> Robot:
    # TODO(Steven): Consider just using the make_device_from_device_class for all types
    if config.type == "koch_follower":
        from .koch_follower import KochFollower

        return KochFollower(config)
    elif config.type == "omx_follower":
        from .omx_follower import OmxFollower

        return OmxFollower(config)
    elif config.type == "so100_follower":
        from .so_follower import SO100Follower

        return SO100Follower(config)
    elif config.type == "so101_follower":
        from .so_follower import SO101Follower

        return SO101Follower(config)
    elif config.type == "lekiwi":
        from .lekiwi import LeKiwi

        return LeKiwi(config)
    elif config.type == "hope_jr_hand":
        from .hope_jr import HopeJrHand

        return HopeJrHand(config)
    elif config.type == "hope_jr_arm":
        from .hope_jr import HopeJrArm

        return HopeJrArm(config)
    elif config.type == "bi_so_follower":
        from .bi_so_follower import BiSOFollower

        return BiSOFollower(config)
    elif config.type == "reachy2":
        from .reachy2 import Reachy2Robot

        return Reachy2Robot(config)
    elif config.type == "openarm_follower":
        from .openarm_follower import OpenArmFollower

        return OpenArmFollower(config)
    elif config.type == "piper_follower":
        from .piper_follower import PiperFollower

        return PiperFollower(config)
    elif config.type == "piperx_follower":
        from .piper_follower import PiperXFollower

        return PiperXFollower(config)
    elif config.type in {"bi_piper_follower", "bi_piperx_follower"}:
        from .bi_piper_follower import BiPiperFollower, BiPiperXFollower

        return BiPiperXFollower(config) if config.type == "bi_piperx_follower" else BiPiperFollower(config)
    elif config.type == "cobot_magic_follower":
        from .cobot_magic import CobotMagicFollower

        return CobotMagicFollower(config)
    elif config.type == "bi_openarm_follower":
        from .bi_openarm_follower import BiOpenArmFollower

        return BiOpenArmFollower(config)
    elif config.type == "mock_robot":
        from tests.mocks.mock_robot import MockRobot

        return MockRobot(config)
    else:
        try:
            return cast(Robot, make_device_from_device_class(config))
        except Exception as e:
            raise ValueError(f"Error creating robot with config {config}: {e}") from e
