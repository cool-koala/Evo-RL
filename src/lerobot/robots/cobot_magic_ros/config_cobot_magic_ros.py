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

    left_state_topic: str = "/cobot_magic/puppet/joint_left"
    right_state_topic: str = "/cobot_magic/puppet/joint_right"
    left_ee_state_topic: str = "/cobot_magic/puppet/end_left"
    right_ee_state_topic: str = "/cobot_magic/puppet/end_right"
    left_command_topic: str = "/cobot_magic/command/joint_left"
    right_command_topic: str = "/cobot_magic/command/joint_right"
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

    def __post_init__(self):
        super().__post_init__()
        if self.left_state_topic == self.right_state_topic:
            raise ValueError("Cobot Magic ROS left and right state topics must be different.")
        if self.left_ee_state_topic == self.right_ee_state_topic:
            raise ValueError("Cobot Magic ROS left and right EE state topics must be different.")
        if self.left_command_topic == self.right_command_topic:
            raise ValueError("Cobot Magic ROS left and right command topics must be different.")
        if self.read_timeout_s < 0:
            raise ValueError("`read_timeout_s` must be >= 0.")
        if self.poll_interval_s <= 0:
            raise ValueError("`poll_interval_s` must be > 0.")
        for camera_name, camera in self.cameras.items():
            if not camera.topic:
                raise ValueError(f"Cobot Magic ROS camera {camera_name!r} must define a topic.")
            if camera.height <= 0 or camera.width <= 0 or camera.fps <= 0 or camera.channels != 3:
                raise ValueError(f"Cobot Magic ROS camera {camera_name!r} must be an RGB image shape.")
