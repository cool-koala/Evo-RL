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

from dataclasses import dataclass
from typing import Any

import numpy as np

from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.arx5_sdk import ARX5_ACTION_KEYS, ARX5_GRIPPER_KEY, ARX5_JOINT_ACTION_KEYS

ROS_JOINT_NAMES = tuple(f"joint{idx}" for idx in range(7))


@dataclass
class RosImports:
    rospy: Any
    Header: Any
    Bool: Any
    JointState: Any
    Image: Any


def import_ros() -> RosImports:
    """Lazy import ROS1 modules so non-ROS tests and config parsing keep working."""

    try:
        import rospy
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Bool, Header
    except ImportError as exc:
        raise ImportError(
            "Cobot Magic ROS backend requires ROS1 Python packages. "
            "Source your ROS environment first (`source /opt/ros/noetic/setup.zsh` for zsh, "
            "or `source /opt/ros/noetic/setup.bash` for bash), and install the cobot_magic extra "
            "so `rospkg` and `catkin_pkg` are available."
        ) from exc
    return RosImports(rospy=rospy, Header=Header, Bool=Bool, JointState=JointState, Image=Image)


def ensure_ros_node(rospy: Any, node_name: str) -> None:
    """Initialize a ROS node only if the current process has not done so already."""

    is_initialized = False
    core = getattr(rospy, "core", None)
    if core is not None and hasattr(core, "is_initialized"):
        is_initialized = bool(core.is_initialized())
    elif hasattr(rospy, "get_node_uri"):
        is_initialized = rospy.get_node_uri() is not None

    if not is_initialized:
        rospy.init_node(node_name, anonymous=True, disable_signals=True)


def prefixed_action_features(prefix: str, sync_gripper: bool = True) -> dict[str, type]:
    keys = ARX5_ACTION_KEYS if sync_gripper else ARX5_JOINT_ACTION_KEYS
    return {f"{prefix}_{key}": float for key in keys}


def prefixed_observation_features(prefix: str) -> dict[str, type]:
    features: dict[str, type] = {}
    for key in ARX5_JOINT_ACTION_KEYS:
        joint_name = key.removesuffix(".pos")
        features[f"{prefix}_{joint_name}.pos"] = float
        features[f"{prefix}_{joint_name}.vel"] = float
        features[f"{prefix}_{joint_name}.torque"] = float
    features[f"{prefix}_gripper.pos"] = float
    features[f"{prefix}_gripper.vel"] = float
    features[f"{prefix}_gripper.torque"] = float
    return features


def _read_sequence_value(values: Any, index: int, default: float = 0.0) -> float:
    if values is None or len(values) <= index:
        return default
    return float(values[index])


def joint_state_to_observation(msg: Any, prefix: str) -> RobotObservation:
    obs: RobotObservation = {}
    for idx, key in enumerate(ARX5_JOINT_ACTION_KEYS):
        joint_name = key.removesuffix(".pos")
        obs[f"{prefix}_{joint_name}.pos"] = _read_sequence_value(msg.position, idx)
        obs[f"{prefix}_{joint_name}.vel"] = _read_sequence_value(msg.velocity, idx)
        obs[f"{prefix}_{joint_name}.torque"] = _read_sequence_value(msg.effort, idx)
    obs[f"{prefix}_gripper.pos"] = _read_sequence_value(msg.position, 6)
    obs[f"{prefix}_gripper.vel"] = _read_sequence_value(msg.velocity, 6)
    obs[f"{prefix}_gripper.torque"] = _read_sequence_value(msg.effort, 6)
    return obs


def joint_state_to_action(msg: Any, prefix: str, sync_gripper: bool = True) -> RobotAction:
    action: RobotAction = {}
    for idx, key in enumerate(ARX5_JOINT_ACTION_KEYS):
        action[f"{prefix}_{key}"] = _read_sequence_value(msg.position, idx)
    if sync_gripper:
        action[f"{prefix}_{ARX5_GRIPPER_KEY}"] = _read_sequence_value(msg.position, 6)
    return action


def build_ros_joint_positions(
    action: RobotAction,
    prefix: str,
    *,
    current_state: Any | None,
    sync_gripper: bool,
) -> tuple[list[float], RobotAction]:
    """Convert LeRobot action keys into a ROS JointState position vector."""

    sent: RobotAction = {}
    positions: list[float] = []
    missing = []
    for key in ARX5_JOINT_ACTION_KEYS:
        prefixed_key = f"{prefix}_{key}"
        if prefixed_key not in action:
            missing.append(prefixed_key)
            continue
        value = float(action[prefixed_key])
        positions.append(value)
        sent[prefixed_key] = value

    if missing:
        raise KeyError(f"Cobot Magic ROS action is missing required joint keys: {missing}")

    gripper_key = f"{prefix}_{ARX5_GRIPPER_KEY}"
    if sync_gripper:
        if gripper_key not in action:
            raise KeyError(f"Cobot Magic ROS action is missing required gripper key: {gripper_key}")
        gripper_pos = float(action[gripper_key])
        sent[gripper_key] = gripper_pos
    else:
        gripper_pos = _read_sequence_value(getattr(current_state, "position", None), 6)
    positions.append(gripper_pos)
    return positions, sent


def make_joint_state_message(ros: RosImports, positions: list[float]) -> Any:
    msg = ros.JointState()
    msg.header = ros.Header()
    msg.header.stamp = ros.rospy.Time.now()
    msg.name = list(ROS_JOINT_NAMES)
    msg.position = [float(value) for value in positions]
    msg.velocity = []
    msg.effort = []
    return msg


def ros_image_to_numpy(msg: Any) -> np.ndarray:
    """Convert common ROS image encodings to RGB uint8 numpy arrays."""

    encoding = str(getattr(msg, "encoding", "")).lower()
    height = int(msg.height)
    width = int(msg.width)
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding == "rgb8":
        return data.reshape(height, width, 3).copy()
    if encoding == "bgr8":
        return data.reshape(height, width, 3)[:, :, ::-1].copy()
    if encoding == "rgba8":
        return data.reshape(height, width, 4)[:, :, :3].copy()
    if encoding == "bgra8":
        return data.reshape(height, width, 4)[:, :, 2::-1].copy()

    raise ValueError(f"Unsupported ROS image encoding for Cobot Magic: {msg.encoding!r}")
