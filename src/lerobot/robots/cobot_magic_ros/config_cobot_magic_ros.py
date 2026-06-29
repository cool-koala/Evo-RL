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
from typing import Literal

from ..config import RobotConfig


@dataclass
class CobotMagicRosCameraConfig:
    topic: str
    height: int = 480
    width: int = 640
    fps: int = 30
    channels: int = 3


def _default_cameras() -> dict[str, CobotMagicRosCameraConfig]:
    return {
        "cam_high": CobotMagicRosCameraConfig("/camera_f/color/image_raw"),
        "cam_left_wrist": CobotMagicRosCameraConfig("/camera_l/color/image_raw"),
        "cam_right_wrist": CobotMagicRosCameraConfig("/camera_r/color/image_raw"),
    }


@RobotConfig.register_subclass("cobot_magic_ros_follower")
@dataclass
class CobotMagicRosFollowerConfig(RobotConfig):
    """Cobot Magic follower backed by the existing ROS/C++ runtime."""

    control_mode: Literal["joint", "ee_pose"] = "joint"
    left_state_topic: str = "/cobot_magic/puppet/joint_left"
    right_state_topic: str = "/cobot_magic/puppet/joint_right"
    left_ee_state_topic: str = "/cobot_magic/puppet/end_left"
    right_ee_state_topic: str = "/cobot_magic/puppet/end_right"
    left_command_topic: str = "/cobot_magic/command/joint_left"
    right_command_topic: str = "/cobot_magic/command/joint_right"
    left_ee_command_topic: str = "/cobot_magic/command/ee_left"
    right_ee_command_topic: str = "/cobot_magic/command/ee_right"
    cameras: dict[str, CobotMagicRosCameraConfig] = field(default_factory=_default_cameras)
    sync_gripper: bool = True
    # For real Cobot Magic teleoperation, the ROS2 C++ follower nodes subscribe directly to leader
    # JointState topics at their 200 Hz control rate. Keep this false while recording demonstrations.
    send_actions: bool = False
    node_name: str = "lerobot_cobot_magic_ros_follower"
    subscriber_queue_size: int = 10
    publisher_queue_size: int = 10
    read_timeout_s: float = 5.0
    poll_interval_s: float = 0.01
    left_arm_enabled: bool = True
    right_arm_enabled: bool = True
    ee_command_timeout_s: float = 0.25
    ee_max_xyz_step_m: float = 0.03
    ee_max_rot_step: float = 0.15
    ee_max_gripper_step: float = 0.05
    ee_workspace_min: tuple[float, float, float] | None = None
    ee_workspace_max: tuple[float, float, float] | None = None

    def __post_init__(self):
        super().__post_init__()
        if self.control_mode not in {"joint", "ee_pose"}:
            raise ValueError("`control_mode` must be either 'joint' or 'ee_pose'.")
        if self.left_state_topic == self.right_state_topic:
            raise ValueError("Cobot Magic ROS left and right state topics must be different.")
        if self.left_ee_state_topic == self.right_ee_state_topic:
            raise ValueError("Cobot Magic ROS left and right EE state topics must be different.")
        if self.left_command_topic == self.right_command_topic:
            raise ValueError("Cobot Magic ROS left and right command topics must be different.")
        if self.left_ee_command_topic == self.right_ee_command_topic:
            raise ValueError("Cobot Magic ROS left and right EE command topics must be different.")
        if self.read_timeout_s < 0:
            raise ValueError("`read_timeout_s` must be >= 0.")
        if self.poll_interval_s <= 0:
            raise ValueError("`poll_interval_s` must be > 0.")
        if self.ee_command_timeout_s <= 0:
            raise ValueError("`ee_command_timeout_s` must be > 0.")
        if self.ee_max_xyz_step_m < 0 or self.ee_max_rot_step < 0 or self.ee_max_gripper_step < 0:
            raise ValueError("EE command step limits must be >= 0.")
        if (self.ee_workspace_min is None) != (self.ee_workspace_max is None):
            raise ValueError("`ee_workspace_min` and `ee_workspace_max` must be provided together.")
        if self.ee_workspace_min is not None and self.ee_workspace_max is not None:
            if len(self.ee_workspace_min) != 3 or len(self.ee_workspace_max) != 3:
                raise ValueError("EE workspace bounds must be xyz triples.")
            if any(lo > hi for lo, hi in zip(self.ee_workspace_min, self.ee_workspace_max, strict=True)):
                raise ValueError("Each EE workspace min value must be <= max value.")
        for camera_name, camera in self.cameras.items():
            if not camera.topic:
                raise ValueError(f"Cobot Magic ROS camera {camera_name!r} must define a topic.")
            if camera.height <= 0 or camera.width <= 0 or camera.fps <= 0 or camera.channels != 3:
                raise ValueError(f"Cobot Magic ROS camera {camera_name!r} must be an RGB image shape.")
