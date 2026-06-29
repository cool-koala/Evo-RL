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

"""ROS2 bridge from Cobot Magic EE command topics to ARX5 Cartesian controllers."""

from __future__ import annotations

import argparse
import logging
import time
from dataclasses import dataclass
from typing import Any

from lerobot.utils.arx5_sdk import (
    CobotMagicArmConfig,
    arx5_eef_state_to_pose,
    arx5_state_to_observation,
    get_arx5_sdk,
    make_arx5_cartesian_controller,
    make_arx5_eef_state,
)
from lerobot.utils.cobot_magic_ros import (
    ROS_EE_COMMAND_NAMES,
    ROS_JOINT_NAMES,
    clamp_ee_pose_step,
    import_ros,
    make_joint_state_message,
    make_pose_stamped_message,
)

logger = logging.getLogger(__name__)


@dataclass
class ArmBridgeConfig:
    prefix: str
    interface: str
    command_topic: str
    joint_state_topic: str
    ee_state_topic: str
    enabled: bool = True


@dataclass
class CartesianBridgeConfig:
    left: ArmBridgeConfig
    right: ArmBridgeConfig
    model: str = "X5"
    node_name: str = "cobot_magic_arx5_cartesian_bridge"
    log_level: str = "WARNING"
    publish_hz: float = 30.0
    command_timeout_s: float = 0.25
    max_xyz_step_m: float = 0.03
    max_rot_step: float = 0.15
    max_gripper_step: float = 0.05
    workspace_min: tuple[float, float, float] | None = None
    workspace_max: tuple[float, float, float] | None = None
    set_damping_on_disconnect: bool = True


class _ArmCartesianBridge:
    def __init__(self, *, config: ArmBridgeConfig, bridge_config: CartesianBridgeConfig, ros: Any, node: Any):
        self.config = config
        self.bridge_config = bridge_config
        self.ros = ros
        self.node = node
        self.arx5 = get_arx5_sdk()
        self.controller: Any | None = None
        self.last_command_time: float | None = None
        self.last_target: list[float] | None = None
        self.active = False
        self.command_subscriber: Any | None = None
        self.joint_state_publisher: Any | None = None
        self.ee_state_publisher: Any | None = None

    def connect(self) -> None:
        arm_config = CobotMagicArmConfig(
            model=self.bridge_config.model,
            interface=self.config.interface,
            controller_type="cartesian_controller",
            log_level=self.bridge_config.log_level,
            set_damping_on_disconnect=self.bridge_config.set_damping_on_disconnect,
        )
        self.controller = make_arx5_cartesian_controller(arm_config)
        self.command_subscriber = self.node.create_subscription(
            self.ros.JointState,
            self.config.command_topic,
            self._command_callback,
            10,
        )
        self.joint_state_publisher = self.node.create_publisher(
            self.ros.JointState,
            self.config.joint_state_topic,
            10,
        )
        self.ee_state_publisher = self.node.create_publisher(
            self.ros.PoseStamped,
            self.config.ee_state_topic,
            10,
        )
        logger.info(
            "Connected %s Cartesian bridge interface=%s command_topic=%s.",
            self.config.prefix,
            self.config.interface,
            self.config.command_topic,
        )

    def _require_controller(self) -> Any:
        if self.controller is None:
            raise RuntimeError(f"{self.config.prefix} Cartesian bridge is not connected.")
        return self.controller

    def _current_ee_pose(self) -> list[float]:
        state = self._require_controller().get_eef_state()
        pose = arx5_eef_state_to_pose(state)
        return [
            pose["ee.x"],
            pose["ee.y"],
            pose["ee.z"],
            pose["ee.wx"],
            pose["ee.wy"],
            pose["ee.wz"],
            pose["ee.gripper_pos"],
        ]

    def _command_callback(self, msg: Any) -> None:
        if len(msg.position) < len(ROS_EE_COMMAND_NAMES):
            logger.warning(
                "Ignoring %s EE command with %d positions; expected 7.",
                self.config.prefix,
                len(msg.position),
            )
            return

        now = time.monotonic()
        target = [float(value) for value in msg.position[: len(ROS_EE_COMMAND_NAMES)]]
        current = self.last_target if self.last_target is not None else self._current_ee_pose()
        target = clamp_ee_pose_step(
            target,
            current,
            max_xyz_step_m=self.bridge_config.max_xyz_step_m,
            max_rot_step=self.bridge_config.max_rot_step,
            max_gripper_step=self.bridge_config.max_gripper_step,
            workspace_min=self.bridge_config.workspace_min,
            workspace_max=self.bridge_config.workspace_max,
        )
        cmd = make_arx5_eef_state(self.arx5, target)
        self._require_controller().set_eef_cmd(cmd)
        self.last_command_time = now
        self.last_target = target
        self.active = True

    def publish_state(self) -> None:
        controller = self._require_controller()
        joint_state = controller.get_joint_state()
        joint_obs = arx5_state_to_observation(joint_state)
        joint_positions = [
            joint_obs["joint_1.pos"],
            joint_obs["joint_2.pos"],
            joint_obs["joint_3.pos"],
            joint_obs["joint_4.pos"],
            joint_obs["joint_5.pos"],
            joint_obs["joint_6.pos"],
            joint_obs["gripper.pos"],
        ]
        self.joint_state_publisher.publish(
            make_joint_state_message(self.ros, joint_positions, names=ROS_JOINT_NAMES, node=self.node)
        )

        ee_pose = self._current_ee_pose()
        self.ee_state_publisher.publish(make_pose_stamped_message(self.ros, ee_pose, node=self.node))

    def enforce_timeout(self) -> None:
        if not self.active or self.last_command_time is None:
            return
        if time.monotonic() - self.last_command_time <= self.bridge_config.command_timeout_s:
            return
        self.active = False
        self.last_target = self._current_ee_pose()
        self._require_controller().set_eef_cmd(make_arx5_eef_state(self.arx5, self.last_target))
        logger.warning(
            "%s EE command timed out after %.3fs; holding current pose.",
            self.config.prefix,
            self.bridge_config.command_timeout_s,
        )

    def disconnect(self) -> None:
        try:
            if self.controller is not None and self.bridge_config.set_damping_on_disconnect:
                self.controller.set_to_damping()
        finally:
            if self.command_subscriber is not None:
                self.node.destroy_subscription(self.command_subscriber)
            for publisher in (self.joint_state_publisher, self.ee_state_publisher):
                if publisher is not None:
                    self.node.destroy_publisher(publisher)
            self.controller = None
            self.command_subscriber = None
            self.joint_state_publisher = None
            self.ee_state_publisher = None
            self.active = False


class CobotMagicCartesianBridge:
    def __init__(self, config: CartesianBridgeConfig):
        self.config = config
        self.ros = import_ros()
        if not self.ros.rclpy.ok():
            self.ros.rclpy.init(args=None, signal_handler_options=None)
        self.node = self.ros.node(config.node_name)
        self.arms: list[_ArmCartesianBridge] = []
        for arm_config in (config.left, config.right):
            if arm_config.enabled:
                self.arms.append(
                    _ArmCartesianBridge(config=arm_config, bridge_config=config, ros=self.ros, node=self.node)
                )

    def connect(self) -> None:
        if not self.arms:
            raise ValueError("At least one Cobot Magic arm must be enabled for the Cartesian bridge.")
        for arm in self.arms:
            arm.connect()

    def spin(self) -> None:
        if self.config.publish_hz <= 0:
            raise ValueError("`publish_hz` must be > 0.")
        self.connect()
        period_s = 1.0 / self.config.publish_hz
        logger.info("Cobot Magic Cartesian bridge running with %d enabled arm(s).", len(self.arms))
        try:
            while self.ros.rclpy.ok():
                loop_start = time.monotonic()
                self.ros.rclpy.spin_once(self.node, timeout_sec=0.0)
                for arm in self.arms:
                    arm.enforce_timeout()
                    arm.publish_state()
                time.sleep(max(0.0, period_s - (time.monotonic() - loop_start)))
        finally:
            self.disconnect()

    def disconnect(self) -> None:
        for arm in self.arms:
            arm.disconnect()
        self.node.destroy_node()
        if self.ros.rclpy.ok():
            self.ros.rclpy.shutdown()


def _parse_xyz_bounds(value: str) -> tuple[float, float, float] | None:
    if value == "":
        return None
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("Workspace bounds must be comma-separated xyz triples.")
    return tuple(float(part) for part in parts)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Cobot Magic ARX5 Cartesian ROS2 bridge.")
    parser.add_argument("--left-interface", default="can3")
    parser.add_argument("--right-interface", default="can1")
    parser.add_argument("--left-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--right-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--model", default="X5")
    parser.add_argument("--publish-hz", type=float, default=30.0)
    parser.add_argument("--command-timeout-s", type=float, default=0.25)
    parser.add_argument("--max-xyz-step-m", type=float, default=0.03)
    parser.add_argument("--max-rot-step", type=float, default=0.15)
    parser.add_argument("--max-gripper-step", type=float, default=0.05)
    parser.add_argument("--workspace-min", type=_parse_xyz_bounds, default=None)
    parser.add_argument("--workspace-max", type=_parse_xyz_bounds, default=None)
    parser.add_argument("--log-level", default="INFO")
    parser.add_argument("--arx-log-level", default="WARNING")
    parser.add_argument("--set-damping-on-disconnect", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def build_config(args: argparse.Namespace) -> CartesianBridgeConfig:
    return CartesianBridgeConfig(
        left=ArmBridgeConfig(
            prefix="left",
            interface=args.left_interface,
            command_topic="/cobot_magic/command/ee_left",
            joint_state_topic="/cobot_magic/puppet/joint_left",
            ee_state_topic="/cobot_magic/puppet/end_left",
            enabled=args.left_enabled,
        ),
        right=ArmBridgeConfig(
            prefix="right",
            interface=args.right_interface,
            command_topic="/cobot_magic/command/ee_right",
            joint_state_topic="/cobot_magic/puppet/joint_right",
            ee_state_topic="/cobot_magic/puppet/end_right",
            enabled=args.right_enabled,
        ),
        model=args.model,
        publish_hz=args.publish_hz,
        command_timeout_s=args.command_timeout_s,
        max_xyz_step_m=args.max_xyz_step_m,
        max_rot_step=args.max_rot_step,
        max_gripper_step=args.max_gripper_step,
        workspace_min=args.workspace_min,
        workspace_max=args.workspace_max,
        log_level=args.arx_log_level,
        set_damping_on_disconnect=args.set_damping_on_disconnect,
    )


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(message)s",
        force=True,
    )
    if (args.workspace_min is None) != (args.workspace_max is None):
        raise ValueError("--workspace-min and --workspace-max must be provided together.")
    bridge = CobotMagicCartesianBridge(build_config(args))
    bridge.spin()


if __name__ == "__main__":
    main()
