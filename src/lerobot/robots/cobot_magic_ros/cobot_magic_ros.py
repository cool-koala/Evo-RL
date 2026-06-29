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
import time
from functools import cached_property
from typing import Any

from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.cobot_magic_ros import (
    ROS_EE_COMMAND_NAMES,
    build_ros_ee_positions,
    build_ros_joint_positions,
    ensure_ros_node,
    import_ros,
    joint_state_to_observation,
    make_joint_state_message,
    pose_stamped_to_ee_pose,
    prefixed_action_features,
    prefixed_ee_pose_features,
    prefixed_observation_features,
    ros_image_to_numpy,
    spin_ros_once,
)
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from .config_cobot_magic_ros import CobotMagicRosFollowerConfig

logger = logging.getLogger(__name__)


class CobotMagicRosFollower(Robot):
    """Cobot Magic follower using ROS topics from the existing C++ control stack."""

    config_class = CobotMagicRosFollowerConfig
    name = "cobot_magic_ros_follower"

    def __init__(self, config: CobotMagicRosFollowerConfig):
        super().__init__(config)
        self.config = config
        self.cameras = config.cameras
        self._ros = None
        self._ros_node: Any | None = None
        self._left_state: Any | None = None
        self._right_state: Any | None = None
        self._left_ee_state: Any | None = None
        self._right_ee_state: Any | None = None
        self._images: dict[str, Any] = {}
        self._subscribers: list[Any] = []
        self._left_command_publisher: Any | None = None
        self._right_command_publisher: Any | None = None
        self._left_ee_command_publisher: Any | None = None
        self._right_ee_command_publisher: Any | None = None
        self._is_connected = False

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {
            **prefixed_observation_features("left"),
            **prefixed_observation_features("right"),
            **prefixed_ee_pose_features("left"),
            **prefixed_ee_pose_features("right"),
        }
        for camera_name, camera in self.config.cameras.items():
            features[camera_name] = (camera.height, camera.width, camera.channels)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            **prefixed_action_features("left", self.config.sync_gripper),
            **prefixed_action_features("right", self.config.sync_gripper),
            **prefixed_ee_pose_features("left"),
            **prefixed_ee_pose_features("right"),
        }

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        self._ros = import_ros()
        self._ros_node = ensure_ros_node(self._ros, self.config.node_name)

        self._left_command_publisher = self._ros_node.create_publisher(
            self._ros.JointState,
            self.config.left_command_topic,
            self.config.publisher_queue_size,
        )
        self._right_command_publisher = self._ros_node.create_publisher(
            self._ros.JointState,
            self.config.right_command_topic,
            self.config.publisher_queue_size,
        )
        self._left_ee_command_publisher = self._ros_node.create_publisher(
            self._ros.JointState,
            self.config.left_ee_command_topic,
            self.config.publisher_queue_size,
        )
        self._right_ee_command_publisher = self._ros_node.create_publisher(
            self._ros.JointState,
            self.config.right_ee_command_topic,
            self.config.publisher_queue_size,
        )

        self._subscribers = [
            self._ros_node.create_subscription(
                self._ros.JointState,
                self.config.left_state_topic,
                self._left_state_callback,
                self.config.subscriber_queue_size,
            ),
            self._ros_node.create_subscription(
                self._ros.JointState,
                self.config.right_state_topic,
                self._right_state_callback,
                self.config.subscriber_queue_size,
            ),
            self._ros_node.create_subscription(
                self._ros.PoseStamped,
                self.config.left_ee_state_topic,
                self._left_ee_state_callback,
                self.config.subscriber_queue_size,
            ),
            self._ros_node.create_subscription(
                self._ros.PoseStamped,
                self.config.right_ee_state_topic,
                self._right_ee_state_callback,
                self.config.subscriber_queue_size,
            ),
        ]
        for camera_name, camera in self.config.cameras.items():
            self._subscribers.append(
                self._ros_node.create_subscription(
                    self._ros.Image,
                    camera.topic,
                    lambda msg, camera_name=camera_name: self._image_callback(camera_name, msg),
                    self._ros.qos_profile_sensor_data,
                )
            )

        self._is_connected = True
        logger.info("%s connected.", self)

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        logger.info("Cobot Magic ROS backend uses the external ROS runtime calibration.")

    def configure(self) -> None:
        return

    def setup_motors(self) -> None:
        raise NotImplementedError("Cobot Magic ROS motors are configured by the external ROS runtime.")

    def _left_state_callback(self, msg: Any) -> None:
        self._left_state = msg

    def _right_state_callback(self, msg: Any) -> None:
        self._right_state = msg

    def _left_ee_state_callback(self, msg: Any) -> None:
        self._left_ee_state = msg

    def _right_ee_state_callback(self, msg: Any) -> None:
        self._right_ee_state = msg

    def _image_callback(self, camera_name: str, msg: Any) -> None:
        self._images[camera_name] = msg

    def _wait_for(self, predicate) -> None:
        deadline = time.perf_counter() + self.config.read_timeout_s
        while not predicate():
            if self.config.read_timeout_s == 0 or time.perf_counter() >= deadline:
                return
            if self._ros is not None:
                spin_ros_once(self._ros, timeout_sec=0.0)
            time.sleep(self.config.poll_interval_s)

    def _require_state(self, state: Any | None, topic: str, msg_type: str = "JointState") -> Any:
        if state is None:
            raise RuntimeError(f"No Cobot Magic ROS {msg_type} has been received from {topic!r}.")
        return state

    @check_if_not_connected
    def get_joint_observation(self) -> RobotObservation:
        self._wait_for(
            lambda: (
                self._left_state is not None
                and self._right_state is not None
                and self._left_ee_state is not None
                and self._right_ee_state is not None
            )
        )
        left_state = self._require_state(self._left_state, self.config.left_state_topic)
        right_state = self._require_state(self._right_state, self.config.right_state_topic)
        left_ee_state = self._require_state(
            self._left_ee_state, self.config.left_ee_state_topic, "PoseStamped"
        )
        right_ee_state = self._require_state(
            self._right_ee_state, self.config.right_ee_state_topic, "PoseStamped"
        )
        return {
            **joint_state_to_observation(left_state, "left"),
            **joint_state_to_observation(right_state, "right"),
            **pose_stamped_to_ee_pose(left_ee_state, "left"),
            **pose_stamped_to_ee_pose(right_ee_state, "right"),
        }

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        obs = self.get_joint_observation()

        for camera_name in self.config.cameras:
            self._wait_for(lambda camera_name=camera_name: camera_name in self._images)
            image_msg = self._images.get(camera_name)
            if image_msg is None:
                topic = self.config.cameras[camera_name].topic
                raise RuntimeError(
                    f"No Cobot Magic ROS image has been received for camera {camera_name!r} on {topic!r}. "
                    "Start the camera ROS nodes first, for example "
                    "`third_party/cobot_magic_ros_runtime/tools/cameras.sh`."
                )
            obs[camera_name] = ros_image_to_numpy(image_msg)
        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        return self._send_action(action)

    @check_if_not_connected
    def send_action_without_relative_limit(self, action: RobotAction) -> RobotAction:
        """Send an interpolated/manual action through the same command path."""

        return self._send_action(action)

    def _send_action(self, action: RobotAction) -> RobotAction:
        if self._ros is None:
            raise RuntimeError("Cobot Magic ROS follower is not connected.")
        spin_ros_once(self._ros, timeout_sec=0.0)
        if self.config.control_mode == "ee_pose":
            return self._send_ee_action(action)
        return self._send_joint_action(action)

    def _send_joint_action(self, action: RobotAction) -> RobotAction:
        if not self.config.send_actions:
            _, sent_left = build_ros_joint_positions(
                action,
                "left",
                current_state=self._left_state,
                sync_gripper=self.config.sync_gripper,
            )
            _, sent_right = build_ros_joint_positions(
                action,
                "right",
                current_state=self._right_state,
                sync_gripper=self.config.sync_gripper,
            )
            return {**action, **sent_left, **sent_right}
        left_positions, sent_left = build_ros_joint_positions(
            action,
            "left",
            current_state=self._left_state,
            sync_gripper=self.config.sync_gripper,
        )
        right_positions, sent_right = build_ros_joint_positions(
            action,
            "right",
            current_state=self._right_state,
            sync_gripper=self.config.sync_gripper,
        )
        self._left_command_publisher.publish(make_joint_state_message(self._ros, left_positions))
        self._right_command_publisher.publish(make_joint_state_message(self._ros, right_positions))
        return {**action, **sent_left, **sent_right}

    def _send_ee_action(self, action: RobotAction) -> RobotAction:
        sent_left: RobotAction = {}
        sent_right: RobotAction = {}
        left_positions: list[float] | None = None
        right_positions: list[float] | None = None
        if self.config.left_arm_enabled:
            left_positions, sent_left = build_ros_ee_positions(action, "left")
        if self.config.right_arm_enabled:
            right_positions, sent_right = build_ros_ee_positions(action, "right")
        if not self.config.send_actions:
            return {**action, **sent_left, **sent_right}

        if left_positions is not None:
            self._left_ee_command_publisher.publish(
                make_joint_state_message(
                    self._ros,
                    left_positions,
                    names=ROS_EE_COMMAND_NAMES,
                )
            )
        if right_positions is not None:
            self._right_ee_command_publisher.publish(
                make_joint_state_message(
                    self._ros,
                    right_positions,
                    names=ROS_EE_COMMAND_NAMES,
                )
            )
        return {**action, **sent_left, **sent_right}

    def disconnect(self) -> None:
        try:
            for subscriber in self._subscribers:
                if self._ros_node is not None:
                    self._ros_node.destroy_subscription(subscriber)
            for publisher in (
                self._left_command_publisher,
                self._right_command_publisher,
                self._left_ee_command_publisher,
                self._right_ee_command_publisher,
            ):
                if self._ros_node is not None and publisher is not None:
                    self._ros_node.destroy_publisher(publisher)
        finally:
            self._subscribers = []
            self._left_command_publisher = None
            self._right_command_publisher = None
            self._left_ee_command_publisher = None
            self._right_ee_command_publisher = None
            self._is_connected = False
            logger.info("%s disconnected.", self)
