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
    build_ros_joint_positions,
    ensure_ros_node,
    import_ros,
    joint_state_to_observation,
    make_joint_state_message,
    prefixed_action_features,
    prefixed_observation_features,
    ros_image_to_numpy,
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
        self._left_state: Any | None = None
        self._right_state: Any | None = None
        self._images: dict[str, Any] = {}
        self._subscribers: list[Any] = []
        self._left_command_publisher: Any | None = None
        self._right_command_publisher: Any | None = None
        self._is_connected = False

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = {
            **prefixed_observation_features("left"),
            **prefixed_observation_features("right"),
        }
        for camera_name, camera in self.config.cameras.items():
            features[camera_name] = (camera.height, camera.width, camera.channels)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            **prefixed_action_features("left", self.config.sync_gripper),
            **prefixed_action_features("right", self.config.sync_gripper),
        }

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        self._ros = import_ros()
        ensure_ros_node(self._ros.rospy, self.config.node_name)

        self._left_command_publisher = self._ros.rospy.Publisher(
            self.config.left_command_topic,
            self._ros.JointState,
            queue_size=self.config.publisher_queue_size,
        )
        self._right_command_publisher = self._ros.rospy.Publisher(
            self.config.right_command_topic,
            self._ros.JointState,
            queue_size=self.config.publisher_queue_size,
        )

        self._subscribers = [
            self._ros.rospy.Subscriber(
                self.config.left_state_topic,
                self._ros.JointState,
                self._left_state_callback,
                queue_size=self.config.subscriber_queue_size,
                tcp_nodelay=True,
            ),
            self._ros.rospy.Subscriber(
                self.config.right_state_topic,
                self._ros.JointState,
                self._right_state_callback,
                queue_size=self.config.subscriber_queue_size,
                tcp_nodelay=True,
            ),
        ]
        for camera_name, camera in self.config.cameras.items():
            self._subscribers.append(
                self._ros.rospy.Subscriber(
                    camera.topic,
                    self._ros.Image,
                    lambda msg, camera_name=camera_name: self._image_callback(camera_name, msg),
                    queue_size=self.config.subscriber_queue_size,
                    tcp_nodelay=True,
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

    def _image_callback(self, camera_name: str, msg: Any) -> None:
        self._images[camera_name] = msg

    def _wait_for(self, predicate) -> None:
        deadline = time.perf_counter() + self.config.read_timeout_s
        while not predicate():
            if self.config.read_timeout_s == 0 or time.perf_counter() >= deadline:
                return
            time.sleep(self.config.poll_interval_s)

    def _require_state(self, state: Any | None, topic: str) -> Any:
        if state is None:
            raise RuntimeError(f"No Cobot Magic ROS JointState has been received from {topic!r}.")
        return state

    @check_if_not_connected
    def get_joint_observation(self) -> RobotObservation:
        self._wait_for(lambda: self._left_state is not None and self._right_state is not None)
        left_state = self._require_state(self._left_state, self.config.left_state_topic)
        right_state = self._require_state(self._right_state, self.config.right_state_topic)
        return {
            **joint_state_to_observation(left_state, "left"),
            **joint_state_to_observation(right_state, "right"),
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
        if self._ros is None:
            raise RuntimeError("Cobot Magic ROS follower is not connected.")
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
        return {**sent_left, **sent_right}

    def disconnect(self) -> None:
        try:
            for subscriber in self._subscribers:
                if hasattr(subscriber, "unregister"):
                    subscriber.unregister()
        finally:
            self._subscribers = []
            self._left_command_publisher = None
            self._right_command_publisher = None
            self._is_connected = False
            logger.info("%s disconnected.", self)
