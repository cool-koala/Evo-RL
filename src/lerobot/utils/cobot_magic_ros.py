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

import threading
from dataclasses import dataclass
from typing import Any

import numpy as np

from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.arx5_sdk import ARX5_ACTION_KEYS, ARX5_GRIPPER_KEY, ARX5_JOINT_ACTION_KEYS

ROS_JOINT_NAMES = tuple(f"joint{idx}" for idx in range(7))


@dataclass
class RosImports:
    rclpy: Any
    node: Any
    qos_profile_sensor_data: Any
    Header: Any
    Bool: Any
    JointState: Any
    PoseStamped: Any
    Image: Any


def import_ros() -> RosImports:
    """Lazy import ROS2 modules so non-ROS tests and config parsing keep working."""

    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Bool, Header
    except ImportError as exc:
        raise ImportError(
            "Cobot Magic ROS backend requires ROS2 Python packages. "
            "Source your ROS2 Jazzy environment first (`source /opt/ros/jazzy/setup.bash`)."
        ) from exc
    return RosImports(
        rclpy=rclpy,
        node=Node,
        qos_profile_sensor_data=qos_profile_sensor_data,
        Header=Header,
        Bool=Bool,
        JointState=JointState,
        PoseStamped=PoseStamped,
        Image=Image,
    )


_ROS2_NODE: Any | None = None
_ROS2_EXECUTOR: Any | None = None
_ROS2_SPIN_THREAD: threading.Thread | None = None


def ensure_ros_node(ros: RosImports, node_name: str) -> Any:
    """Initialize and return the shared ROS2 node used by the Cobot Magic backend."""

    global _ROS2_EXECUTOR, _ROS2_NODE, _ROS2_SPIN_THREAD

    if not ros.rclpy.ok():
        ros.rclpy.init(args=None, signal_handler_options=None)
    if _ROS2_NODE is None:
        _ROS2_NODE = ros.node(node_name)
    if _ROS2_EXECUTOR is None:
        from rclpy.executors import SingleThreadedExecutor

        _ROS2_EXECUTOR = SingleThreadedExecutor()
        _ROS2_EXECUTOR.add_node(_ROS2_NODE)
    if _ROS2_SPIN_THREAD is None or not _ROS2_SPIN_THREAD.is_alive():
        _ROS2_SPIN_THREAD = threading.Thread(
            target=_ROS2_EXECUTOR.spin,
            name="cobot_magic_ros2_executor",
            daemon=True,
        )
        _ROS2_SPIN_THREAD.start()
    return _ROS2_NODE


def spin_ros_once(ros: RosImports, timeout_sec: float = 0.0) -> None:
    """Service ROS2 callbacks for the shared node."""

    if _ROS2_SPIN_THREAD is not None and _ROS2_SPIN_THREAD.is_alive():
        return
    if _ROS2_NODE is not None and ros.rclpy.ok():
        ros.rclpy.spin_once(_ROS2_NODE, timeout_sec=timeout_sec)


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


EE_POSE_KEYS = ("ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz", "ee.gripper_pos")


def prefixed_ee_pose_features(prefix: str) -> dict[str, type]:
    return {f"{prefix}_{key}": float for key in EE_POSE_KEYS}


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


def pose_stamped_to_ee_pose(msg: Any, prefix: str) -> RobotAction:
    """Convert ARX PoseStamped EE feedback to LeRobot EE pose keys.

    The ARX runtime stores End_Effector_Pose[3:6] in orientation.x/y/z and gripper in
    orientation.w, so these fields are treated as wx/wy/wz/gripper_pos values rather
    than a ROS quaternion.
    """

    pose = msg.pose
    return {
        f"{prefix}_ee.x": float(pose.position.x),
        f"{prefix}_ee.y": float(pose.position.y),
        f"{prefix}_ee.z": float(pose.position.z),
        f"{prefix}_ee.wx": float(pose.orientation.x),
        f"{prefix}_ee.wy": float(pose.orientation.y),
        f"{prefix}_ee.wz": float(pose.orientation.z),
        f"{prefix}_ee.gripper_pos": float(pose.orientation.w),
    }


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
    if _ROS2_NODE is None:
        raise RuntimeError("Cobot Magic ROS2 node has not been initialized.")
    msg.header.stamp = _ROS2_NODE.get_clock().now().to_msg()
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
