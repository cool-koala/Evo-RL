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

from lerobot.processor import RobotAction
from lerobot.teleoperators.utils import TeleopEvents
from lerobot.utils.arx5_sdk import ARX5_ACTION_KEYS, ARX5_GRIPPER_KEY, ARX5_JOINT_ACTION_KEYS
from lerobot.utils.cobot_magic_ros import (
    build_ros_joint_positions,
    ensure_ros_node,
    import_ros,
    joint_state_to_action,
    make_joint_state_message,
    pose_stamped_to_ee_pose,
    prefixed_action_features,
    prefixed_ee_pose_features,
    spin_ros_once,
)
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_cobot_magic_ros import CobotMagicRosLeaderConfig

logger = logging.getLogger(__name__)


class CobotMagicRosLeader(Teleoperator):
    """Reads Cobot Magic leader arm states from ROS and exposes LeRobot actions."""

    config_class = CobotMagicRosLeaderConfig
    name = "cobot_magic_ros_leader"

    def __init__(self, config: CobotMagicRosLeaderConfig):
        super().__init__(config)
        self.config = config
        self._ros = None
        self._ros_node: Any | None = None
        self._left_leader_state: Any | None = None
        self._right_leader_state: Any | None = None
        self._left_leader_ee_state: Any | None = None
        self._right_leader_ee_state: Any | None = None
        self._left_follower_state: Any | None = None
        self._right_follower_state: Any | None = None
        self._left_follower_ee_state: Any | None = None
        self._right_follower_ee_state: Any | None = None
        self._subscribers: list[Any] = []
        self._left_command_publisher: Any | None = None
        self._right_command_publisher: Any | None = None
        self._left_manual_control_publisher: Any | None = None
        self._right_manual_control_publisher: Any | None = None
        self._is_connected = False
        self._manual_control_enabled = False
        self._leader_origin: RobotAction | None = None
        self._follower_origin: RobotAction | None = None

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            **prefixed_action_features("left", self.config.sync_gripper),
            **prefixed_action_features("right", self.config.sync_gripper),
            **prefixed_ee_pose_features("left"),
            **prefixed_ee_pose_features("right"),
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return self.action_features

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
            self.config.left_leader_command_topic,
            self.config.publisher_queue_size,
        )
        self._right_command_publisher = self._ros_node.create_publisher(
            self._ros.JointState,
            self.config.right_leader_command_topic,
            self.config.publisher_queue_size,
        )
        self._left_manual_control_publisher = self._ros_node.create_publisher(
            self._ros.Bool,
            self.config.left_leader_manual_control_topic,
            self.config.publisher_queue_size,
        )
        self._right_manual_control_publisher = self._ros_node.create_publisher(
            self._ros.Bool,
            self.config.right_leader_manual_control_topic,
            self.config.publisher_queue_size,
        )
        self._subscribers = [
            self._ros_node.create_subscription(
                self._ros.JointState,
                self.config.left_leader_state_topic,
                self._left_leader_state_callback,
                self.config.subscriber_queue_size,
            ),
            self._ros_node.create_subscription(
                self._ros.JointState,
                self.config.right_leader_state_topic,
                self._right_leader_state_callback,
                self.config.subscriber_queue_size,
            ),
            self._ros_node.create_subscription(
                self._ros.PoseStamped,
                self.config.left_leader_ee_topic,
                self._left_leader_ee_state_callback,
                self.config.subscriber_queue_size,
            ),
            self._ros_node.create_subscription(
                self._ros.PoseStamped,
                self.config.right_leader_ee_topic,
                self._right_leader_ee_state_callback,
                self.config.subscriber_queue_size,
            ),
        ]
        if self.config.relative_takeover:
            self._subscribers.extend(
                [
                    self._ros_node.create_subscription(
                        self._ros.JointState,
                        self.config.left_follower_state_topic,
                        self._left_follower_state_callback,
                        self.config.subscriber_queue_size,
                    ),
                    self._ros_node.create_subscription(
                        self._ros.JointState,
                        self.config.right_follower_state_topic,
                        self._right_follower_state_callback,
                        self.config.subscriber_queue_size,
                    ),
                    self._ros_node.create_subscription(
                        self._ros.PoseStamped,
                        self.config.left_follower_ee_topic,
                        self._left_follower_ee_state_callback,
                        self.config.subscriber_queue_size,
                    ),
                    self._ros_node.create_subscription(
                        self._ros.PoseStamped,
                        self.config.right_follower_ee_topic,
                        self._right_follower_ee_state_callback,
                        self.config.subscriber_queue_size,
                    ),
                ]
            )
        self._is_connected = True
        self.set_manual_control(self.config.manual_control)
        logger.info("%s connected.", self)

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        logger.info("Cobot Magic ROS leader uses the external ROS runtime calibration.")

    def configure(self) -> None:
        return

    def _left_leader_state_callback(self, msg: Any) -> None:
        self._left_leader_state = msg

    def _right_leader_state_callback(self, msg: Any) -> None:
        self._right_leader_state = msg

    def _left_leader_ee_state_callback(self, msg: Any) -> None:
        self._left_leader_ee_state = msg

    def _right_leader_ee_state_callback(self, msg: Any) -> None:
        self._right_leader_ee_state = msg

    def _left_follower_state_callback(self, msg: Any) -> None:
        self._left_follower_state = msg

    def _right_follower_state_callback(self, msg: Any) -> None:
        self._right_follower_state = msg

    def _left_follower_ee_state_callback(self, msg: Any) -> None:
        self._left_follower_ee_state = msg

    def _right_follower_ee_state_callback(self, msg: Any) -> None:
        self._right_follower_ee_state = msg

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

    def _absolute_leader_action(self) -> RobotAction:
        self._wait_for(
            lambda: (
                self._left_leader_state is not None
                and self._right_leader_state is not None
                and self._left_leader_ee_state is not None
                and self._right_leader_ee_state is not None
            )
        )
        left_state = self._require_state(self._left_leader_state, self.config.left_leader_state_topic)
        right_state = self._require_state(self._right_leader_state, self.config.right_leader_state_topic)
        left_ee_state = self._require_state(
            self._left_leader_ee_state, self.config.left_leader_ee_topic, "PoseStamped"
        )
        right_ee_state = self._require_state(
            self._right_leader_ee_state, self.config.right_leader_ee_topic, "PoseStamped"
        )
        return {
            **joint_state_to_action(left_state, "left", self.config.sync_gripper),
            **joint_state_to_action(right_state, "right", self.config.sync_gripper),
            **pose_stamped_to_ee_pose(left_ee_state, "left"),
            **pose_stamped_to_ee_pose(right_ee_state, "right"),
        }

    def _publish_manual_control(self, enabled: bool) -> None:
        if self._ros is None:
            raise RuntimeError("Cobot Magic ROS leader is not connected.")
        msg = self._ros.Bool()
        msg.data = bool(enabled)
        for _ in range(3):
            self._left_manual_control_publisher.publish(msg)
            self._right_manual_control_publisher.publish(msg)
            spin_ros_once(self._ros, timeout_sec=0.0)
            time.sleep(0.05)

    def _follower_action(self) -> RobotAction:
        self._wait_for(
            lambda: (
                self._left_follower_state is not None
                and self._right_follower_state is not None
                and self._left_follower_ee_state is not None
                and self._right_follower_ee_state is not None
            )
        )
        left_state = self._require_state(self._left_follower_state, self.config.left_follower_state_topic)
        right_state = self._require_state(self._right_follower_state, self.config.right_follower_state_topic)
        left_ee_state = self._require_state(
            self._left_follower_ee_state, self.config.left_follower_ee_topic, "PoseStamped"
        )
        right_ee_state = self._require_state(
            self._right_follower_ee_state, self.config.right_follower_ee_topic, "PoseStamped"
        )
        return {
            **joint_state_to_action(left_state, "left", self.config.sync_gripper),
            **joint_state_to_action(right_state, "right", self.config.sync_gripper),
            **pose_stamped_to_ee_pose(left_ee_state, "left"),
            **pose_stamped_to_ee_pose(right_ee_state, "right"),
        }

    @check_if_not_connected
    def get_absolute_action(self) -> RobotAction:
        """Return the leader's absolute joint targets, ignoring relative takeover offsets."""

        return self._absolute_leader_action()

    @check_if_not_connected
    def set_manual_control(self, enabled: bool) -> None:
        self._manual_control_enabled = bool(enabled)
        self._leader_origin = None
        self._follower_origin = None
        self._publish_manual_control(enabled)
        if not enabled or not self.config.relative_takeover:
            return
        self._leader_origin = self._absolute_leader_action()
        try:
            self._follower_origin = self._follower_action()
        except RuntimeError:
            logger.warning(
                "Follower state is unavailable when entering Cobot Magic ROS intervention; "
                "falling back to absolute leader targets."
            )

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        if self._ros is not None:
            spin_ros_once(self._ros, timeout_sec=0.0)
        action = self._absolute_leader_action()
        if (
            self._manual_control_enabled
            and self.config.relative_takeover
            and self._leader_origin is not None
            and self._follower_origin is not None
        ):
            relative_action: RobotAction = {}
            keys = list(ARX5_ACTION_KEYS if self.config.sync_gripper else ARX5_JOINT_ACTION_KEYS)
            keys.extend(("ee.x", "ee.y", "ee.z", "ee.wx", "ee.wy", "ee.wz", "ee.gripper_pos"))
            for key in keys:
                for prefix in ("left", "right"):
                    action_key = f"{prefix}_{key}"
                    relative_action[action_key] = (
                        float(self._follower_origin[action_key])
                        + float(action[action_key])
                        - float(self._leader_origin[action_key])
                    )
            action = relative_action
        if not self.config.sync_gripper:
            action.pop(f"left_{ARX5_GRIPPER_KEY}", None)
            action.pop(f"right_{ARX5_GRIPPER_KEY}", None)
        return action

    def get_teleop_events(self) -> dict[TeleopEvents, bool]:
        return {
            TeleopEvents.IS_INTERVENTION: self.config.always_intervene,
            TeleopEvents.TERMINATE_EPISODE: False,
            TeleopEvents.SUCCESS: False,
            TeleopEvents.RERECORD_EPISODE: False,
        }

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, Any]) -> None:
        if self._ros is None:
            raise RuntimeError("Cobot Magic ROS leader is not connected.")

        self.set_manual_control(False)
        left_positions, _ = build_ros_joint_positions(
            feedback,
            "left",
            current_state=self._left_leader_state,
            sync_gripper=self.config.sync_gripper,
        )
        right_positions, _ = build_ros_joint_positions(
            feedback,
            "right",
            current_state=self._right_leader_state,
            sync_gripper=self.config.sync_gripper,
        )
        self._left_command_publisher.publish(make_joint_state_message(self._ros, left_positions))
        self._right_command_publisher.publish(make_joint_state_message(self._ros, right_positions))

    def disconnect(self) -> None:
        try:
            for subscriber in self._subscribers:
                if self._ros_node is not None:
                    self._ros_node.destroy_subscription(subscriber)
            for publisher in (
                self._left_command_publisher,
                self._right_command_publisher,
                self._left_manual_control_publisher,
                self._right_manual_control_publisher,
            ):
                if self._ros_node is not None and publisher is not None:
                    self._ros_node.destroy_publisher(publisher)
        finally:
            self._subscribers = []
            self._left_command_publisher = None
            self._right_command_publisher = None
            self._left_manual_control_publisher = None
            self._right_manual_control_publisher = None
            self._is_connected = False
            self._manual_control_enabled = False
            self._leader_origin = None
            self._follower_origin = None
            logger.info("%s disconnected.", self)
