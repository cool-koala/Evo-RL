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
        self._left_manual_control_state: bool | None = None
        self._right_manual_control_state: bool | None = None
        self._left_leader_state_received_at: float | None = None
        self._right_leader_state_received_at: float | None = None
        self._left_leader_ee_state_received_at: float | None = None
        self._right_leader_ee_state_received_at: float | None = None
        self._left_follower_state_received_at: float | None = None
        self._right_follower_state_received_at: float | None = None
        self._left_follower_ee_state_received_at: float | None = None
        self._right_follower_ee_state_received_at: float | None = None
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
        try:
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

            subscriptions = (
                (
                    self._ros.JointState,
                    self.config.left_leader_state_topic,
                    self._left_leader_state_callback,
                ),
                (
                    self._ros.JointState,
                    self.config.right_leader_state_topic,
                    self._right_leader_state_callback,
                ),
                (
                    self._ros.PoseStamped,
                    self.config.left_leader_ee_topic,
                    self._left_leader_ee_state_callback,
                ),
                (
                    self._ros.PoseStamped,
                    self.config.right_leader_ee_topic,
                    self._right_leader_ee_state_callback,
                ),
                (
                    self._ros.Bool,
                    self.config.left_leader_manual_control_status_topic,
                    self._left_manual_control_state_callback,
                ),
                (
                    self._ros.Bool,
                    self.config.right_leader_manual_control_status_topic,
                    self._right_manual_control_state_callback,
                ),
            )
            for msg_type, topic, callback in subscriptions:
                self._subscribers.append(
                    self._ros_node.create_subscription(
                        msg_type,
                        topic,
                        callback,
                        self.config.subscriber_queue_size,
                    )
                )
            if self.config.relative_takeover:
                follower_subscriptions = (
                    (
                        self._ros.JointState,
                        self.config.left_follower_state_topic,
                        self._left_follower_state_callback,
                    ),
                    (
                        self._ros.JointState,
                        self.config.right_follower_state_topic,
                        self._right_follower_state_callback,
                    ),
                    (
                        self._ros.PoseStamped,
                        self.config.left_follower_ee_topic,
                        self._left_follower_ee_state_callback,
                    ),
                    (
                        self._ros.PoseStamped,
                        self.config.right_follower_ee_topic,
                        self._right_follower_ee_state_callback,
                    ),
                )
                for msg_type, topic, callback in follower_subscriptions:
                    self._subscribers.append(
                        self._ros_node.create_subscription(
                            msg_type,
                            topic,
                            callback,
                            self.config.subscriber_queue_size,
                        )
                    )

            self._is_connected = True
            self.set_manual_control(self.config.manual_control)
        except Exception:
            self.disconnect()
            raise
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
        self._left_leader_state_received_at = time.monotonic()

    def _right_leader_state_callback(self, msg: Any) -> None:
        self._right_leader_state = msg
        self._right_leader_state_received_at = time.monotonic()

    def _left_leader_ee_state_callback(self, msg: Any) -> None:
        self._left_leader_ee_state = msg
        self._left_leader_ee_state_received_at = time.monotonic()

    def _right_leader_ee_state_callback(self, msg: Any) -> None:
        self._right_leader_ee_state = msg
        self._right_leader_ee_state_received_at = time.monotonic()

    def _left_follower_state_callback(self, msg: Any) -> None:
        self._left_follower_state = msg
        self._left_follower_state_received_at = time.monotonic()

    def _right_follower_state_callback(self, msg: Any) -> None:
        self._right_follower_state = msg
        self._right_follower_state_received_at = time.monotonic()

    def _left_follower_ee_state_callback(self, msg: Any) -> None:
        self._left_follower_ee_state = msg
        self._left_follower_ee_state_received_at = time.monotonic()

    def _right_follower_ee_state_callback(self, msg: Any) -> None:
        self._right_follower_ee_state = msg
        self._right_follower_ee_state_received_at = time.monotonic()

    def _left_manual_control_state_callback(self, msg: Any) -> None:
        self._left_manual_control_state = bool(msg.data)

    def _right_manual_control_state_callback(self, msg: Any) -> None:
        self._right_manual_control_state = bool(msg.data)

    def _wait_for(self, predicate) -> None:
        deadline = time.perf_counter() + self.config.read_timeout_s
        while not predicate():
            if self.config.read_timeout_s == 0 or time.perf_counter() >= deadline:
                return
            if self._ros is not None:
                spin_ros_once(self._ros, timeout_sec=0.0)
            time.sleep(self.config.poll_interval_s)

    def _state_is_fresh(self, state: Any | None, received_at: float | None) -> bool:
        if state is None:
            return False
        if self.config.state_timeout_s <= 0:
            return True
        if received_at is None:
            return False
        return max(0.0, time.monotonic() - received_at) <= self.config.state_timeout_s

    def _require_state(
        self,
        state: Any | None,
        received_at: float | None,
        topic: str,
        msg_type: str = "JointState",
    ) -> Any:
        if state is None:
            raise RuntimeError(f"No Cobot Magic ROS {msg_type} has been received from {topic!r}.")
        if self.config.state_timeout_s > 0:
            if received_at is None:
                raise RuntimeError(f"Cobot Magic ROS {msg_type} from {topic!r} has no receive timestamp.")
            age_s = max(0.0, time.monotonic() - received_at)
            if age_s > self.config.state_timeout_s:
                raise RuntimeError(
                    f"Stale Cobot Magic ROS {msg_type} from {topic!r}: "
                    f"age={age_s:.3f}s timeout={self.config.state_timeout_s:.3f}s."
                )
        return state

    def _absolute_leader_action(self) -> RobotAction:
        self._wait_for(
            lambda: (
                self._state_is_fresh(self._left_leader_state, self._left_leader_state_received_at)
                and self._state_is_fresh(
                    self._right_leader_state,
                    self._right_leader_state_received_at,
                )
                and self._state_is_fresh(
                    self._left_leader_ee_state,
                    self._left_leader_ee_state_received_at,
                )
                and self._state_is_fresh(
                    self._right_leader_ee_state,
                    self._right_leader_ee_state_received_at,
                )
            )
        )
        left_state = self._require_state(
            self._left_leader_state,
            self._left_leader_state_received_at,
            self.config.left_leader_state_topic,
        )
        right_state = self._require_state(
            self._right_leader_state,
            self._right_leader_state_received_at,
            self.config.right_leader_state_topic,
        )
        left_ee_state = self._require_state(
            self._left_leader_ee_state,
            self._left_leader_ee_state_received_at,
            self.config.left_leader_ee_topic,
            "PoseStamped",
        )
        right_ee_state = self._require_state(
            self._right_leader_ee_state,
            self._right_leader_ee_state_received_at,
            self.config.right_leader_ee_topic,
            "PoseStamped",
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

    def _wait_for_manual_control_state(self, enabled: bool) -> None:
        self._wait_for(
            lambda: self._left_manual_control_state is enabled and self._right_manual_control_state is enabled
        )
        if self._left_manual_control_state is not enabled or self._right_manual_control_state is not enabled:
            raise RuntimeError(
                "Cobot Magic leader manual-control mode was not acknowledged by both arms: "
                f"requested={enabled} left={self._left_manual_control_state} "
                f"right={self._right_manual_control_state}."
            )

    def _request_manual_control(self, enabled: bool) -> None:
        # A Bool status has no request id. Clearing the cached values prevents
        # an ACK from an earlier transition from satisfying this request.
        self._left_manual_control_state = None
        self._right_manual_control_state = None
        self._publish_manual_control(enabled)
        self._wait_for_manual_control_state(enabled)

    def _best_effort_disable_manual_control(self) -> bool:
        try:
            self._request_manual_control(False)
        except Exception:
            logger.exception("Failed to roll back Cobot Magic leader manual-control mode.")
            return False
        self._manual_control_enabled = False
        self._leader_origin = None
        self._follower_origin = None
        return True

    def _follower_action(self) -> RobotAction:
        self._wait_for(
            lambda: (
                self._state_is_fresh(self._left_follower_state, self._left_follower_state_received_at)
                and self._state_is_fresh(
                    self._right_follower_state,
                    self._right_follower_state_received_at,
                )
                and self._state_is_fresh(
                    self._left_follower_ee_state,
                    self._left_follower_ee_state_received_at,
                )
                and self._state_is_fresh(
                    self._right_follower_ee_state,
                    self._right_follower_ee_state_received_at,
                )
            )
        )
        left_state = self._require_state(
            self._left_follower_state,
            self._left_follower_state_received_at,
            self.config.left_follower_state_topic,
        )
        right_state = self._require_state(
            self._right_follower_state,
            self._right_follower_state_received_at,
            self.config.right_follower_state_topic,
        )
        left_ee_state = self._require_state(
            self._left_follower_ee_state,
            self._left_follower_ee_state_received_at,
            self.config.left_follower_ee_topic,
            "PoseStamped",
        )
        right_ee_state = self._require_state(
            self._right_follower_ee_state,
            self._right_follower_ee_state_received_at,
            self.config.right_follower_ee_topic,
            "PoseStamped",
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
    def wait_for_fresh_state(self) -> None:
        """Process ROS callbacks until all states needed for takeover are fresh."""

        self._absolute_leader_action()
        if self.config.relative_takeover:
            self._follower_action()

    @check_if_not_connected
    def set_manual_control(self, enabled: bool) -> None:
        leader_origin: RobotAction | None = None
        follower_origin: RobotAction | None = None
        if enabled and self.config.relative_takeover:
            # Validate both origins before changing either arm's hardware mode.
            self.wait_for_fresh_state()
        try:
            self._request_manual_control(enabled)
        except Exception:
            if enabled:
                self._best_effort_disable_manual_control()
            raise
        if enabled and self.config.relative_takeover:
            try:
                # Capture again after both mode ACKs so the relative origin
                # matches the first action generated in manual mode.
                leader_origin = self._absolute_leader_action()
                follower_origin = self._follower_action()
            except Exception:
                self._best_effort_disable_manual_control()
                raise
        self._manual_control_enabled = bool(enabled)
        self._leader_origin = leader_origin
        self._follower_origin = follower_origin

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
        try:
            # Each command callback atomically switches its arm to ROS command
            # mode and installs this message as the first target.
            self._left_manual_control_state = None
            self._right_manual_control_state = None
            self._left_command_publisher.publish(make_joint_state_message(self._ros, left_positions))
            self._right_command_publisher.publish(make_joint_state_message(self._ros, right_positions))
            self._wait_for_manual_control_state(False)
        except Exception:
            self._best_effort_disable_manual_control()
            raise
        self._manual_control_enabled = False
        self._leader_origin = None
        self._follower_origin = None

    def disconnect(self) -> None:
        try:
            if self._is_connected and (
                self._manual_control_enabled
                or self._left_manual_control_state is True
                or self._right_manual_control_state is True
            ):
                try:
                    self._request_manual_control(False)
                except Exception:
                    logger.exception(
                        "Failed to disable Cobot Magic leader manual control while disconnecting."
                    )
            for subscriber in self._subscribers:
                if self._ros_node is not None:
                    try:
                        self._ros_node.destroy_subscription(subscriber)
                    except Exception:
                        logger.exception("Failed to destroy a Cobot Magic leader ROS subscription.")
            for publisher in (
                self._left_command_publisher,
                self._right_command_publisher,
                self._left_manual_control_publisher,
                self._right_manual_control_publisher,
            ):
                if self._ros_node is not None and publisher is not None:
                    try:
                        self._ros_node.destroy_publisher(publisher)
                    except Exception:
                        logger.exception("Failed to destroy a Cobot Magic leader ROS publisher.")
        finally:
            self._subscribers = []
            self._left_command_publisher = None
            self._right_command_publisher = None
            self._left_manual_control_publisher = None
            self._right_manual_control_publisher = None
            self._left_leader_state = None
            self._right_leader_state = None
            self._left_leader_ee_state = None
            self._right_leader_ee_state = None
            self._left_follower_state = None
            self._right_follower_state = None
            self._left_follower_ee_state = None
            self._right_follower_ee_state = None
            self._left_leader_state_received_at = None
            self._right_leader_state_received_at = None
            self._left_leader_ee_state_received_at = None
            self._right_leader_ee_state_received_at = None
            self._left_follower_state_received_at = None
            self._right_follower_state_received_at = None
            self._left_follower_ee_state_received_at = None
            self._right_follower_ee_state_received_at = None
            self._is_connected = False
            self._manual_control_enabled = False
            self._leader_origin = None
            self._follower_origin = None
            self._left_manual_control_state = None
            self._right_manual_control_state = None
            logger.info("%s disconnected.", self)
