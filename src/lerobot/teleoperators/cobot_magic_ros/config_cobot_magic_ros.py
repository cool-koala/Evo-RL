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

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("cobot_magic_ros_leader")
@dataclass
class CobotMagicRosLeaderConfig(TeleoperatorConfig):
    """Cobot Magic leader backed by ROS JointState topics."""

    left_leader_state_topic: str = "/cobot_magic/leader/joint_left"
    right_leader_state_topic: str = "/cobot_magic/leader/joint_right"
    left_leader_ee_topic: str = "/cobot_magic/leader/end_left"
    right_leader_ee_topic: str = "/cobot_magic/leader/end_right"
    left_follower_ee_topic: str = "/cobot_magic/puppet/end_left"
    right_follower_ee_topic: str = "/cobot_magic/puppet/end_right"
    left_leader_command_topic: str = "/cobot_magic/leader/command_joint_left"
    right_leader_command_topic: str = "/cobot_magic/leader/command_joint_right"
    left_leader_manual_control_topic: str = "/cobot_magic/leader/manual_control_left"
    right_leader_manual_control_topic: str = "/cobot_magic/leader/manual_control_right"
    left_leader_manual_control_status_topic: str = "/cobot_magic/leader/manual_control_status_left"
    right_leader_manual_control_status_topic: str = "/cobot_magic/leader/manual_control_status_right"
    left_follower_state_topic: str = "/cobot_magic/puppet/joint_left"
    right_follower_state_topic: str = "/cobot_magic/puppet/joint_right"
    sync_gripper: bool = True
    relative_takeover: bool = True
    manual_control: bool = True
    startup_sync: bool = False
    startup_sync_duration_s: float = 3.0
    startup_sync_max_joint_delta: float | None = 1.5
    always_intervene: bool = False
    node_name: str = "lerobot_cobot_magic_ros_leader"
    subscriber_queue_size: int = 10
    publisher_queue_size: int = 10
    read_timeout_s: float = 1.0
    poll_interval_s: float = 0.01
    state_timeout_s: float = 0.5

    def __post_init__(self):
        if self.left_leader_state_topic == self.right_leader_state_topic:
            raise ValueError("Cobot Magic ROS left and right leader topics must be different.")
        if self.left_leader_ee_topic == self.right_leader_ee_topic:
            raise ValueError("Cobot Magic ROS left and right leader EE topics must be different.")
        if self.relative_takeover and self.left_follower_ee_topic == self.right_follower_ee_topic:
            raise ValueError("Cobot Magic ROS left and right follower EE topics must be different.")
        if self.left_leader_command_topic == self.right_leader_command_topic:
            raise ValueError("Cobot Magic ROS left and right leader command topics must be different.")
        if self.left_leader_manual_control_topic == self.right_leader_manual_control_topic:
            raise ValueError("Cobot Magic ROS left and right leader manual-control topics must be different.")
        if self.left_leader_manual_control_status_topic == self.right_leader_manual_control_status_topic:
            raise ValueError(
                "Cobot Magic ROS left and right leader manual-control status topics must be different."
            )
        if self.relative_takeover and self.left_follower_state_topic == self.right_follower_state_topic:
            raise ValueError("Cobot Magic ROS left and right follower topics must be different.")
        if self.read_timeout_s < 0:
            raise ValueError("`read_timeout_s` must be >= 0.")
        if self.poll_interval_s <= 0:
            raise ValueError("`poll_interval_s` must be > 0.")
        if self.state_timeout_s < 0:
            raise ValueError("`state_timeout_s` must be >= 0.")
        if self.startup_sync_duration_s <= 0:
            raise ValueError("`startup_sync_duration_s` must be > 0.")
        if self.startup_sync_max_joint_delta is not None and self.startup_sync_max_joint_delta <= 0:
            raise ValueError("`startup_sync_max_joint_delta` must be > 0 when set.")
