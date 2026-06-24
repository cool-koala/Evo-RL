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

import importlib
from dataclasses import dataclass
from types import ModuleType
from typing import Any

import numpy as np

from lerobot.processor import RobotAction, RobotObservation

# ARX-5 SDK 的关节控制器只控制 6 个手臂关节，夹爪通过 JointState.gripper_pos 单独传递。
ARX5_JOINT_NAMES = tuple(f"joint_{idx}" for idx in range(1, 7))
ARX5_JOINT_ACTION_KEYS = tuple(f"{joint}.pos" for joint in ARX5_JOINT_NAMES)
ARX5_GRIPPER_KEY = "gripper.pos"
ARX5_ACTION_KEYS = (*ARX5_JOINT_ACTION_KEYS, ARX5_GRIPPER_KEY)


@dataclass
class CobotMagicArmConfig:
    """单条 ARX-5 机械臂的底层 SDK 配置。"""

    # ARX SDK 支持 X5/L5 等型号；Cobot Magic 当前使用 ARX-5，所以默认固定为 X5。
    model: str = "X5"
    # CAN 或 EtherCAT-CAN 接口名，例如 can0、can1 或 enx...。
    interface: str = "can0"
    # 目前按 Piper 风格使用关节空间控制器，不直接暴露笛卡尔控制器。
    controller_type: str = "joint_controller"
    # 后台收发线程让 set_joint_cmd 近似非阻塞，和 SDK 示例保持一致。
    background_send_recv: bool = True
    # 重力补偿由 ARX SDK/控制器承担；Leader 示教阻尼模式强依赖这个开关。
    gravity_compensation: bool = True
    # SDK 控制周期。None 表示使用 SDK 默认值。
    controller_dt: float | None = None
    # SDK 日志级别，对应 arx5_interface.LogLevel。
    log_level: str = "WARNING"
    # 是否同步夹爪；关闭后只控制 6 个手臂关节。
    sync_gripper: bool = True
    # 连接时是否回 home。真实机器人上自动回零可能有风险，所以默认关闭。
    reset_to_home_on_connect: bool = False
    # 断开前切到阻尼/被动状态，降低机械臂保持力矩带来的安全风险。
    set_damping_on_disconnect: bool = True
    # SDK 是否在对象析构时切 passive；不同版本字段可能存在，设置时做兼容检查。
    shutdown_to_passive: bool = True


def get_arx5_sdk() -> ModuleType:
    """懒加载 ARX-5 Python SDK，并给出面向用户的安装提示。"""

    try:
        return importlib.import_module("arx5_interface")
    except ImportError as exc:
        raise ImportError(
            "Cobot Magic/ARX-5 support requires the ARX5 Python SDK. "
            "Install it with `pip install arx5-interface` or `pip install -e '.[cobot_magic]'`."
        ) from exc


def validate_cobot_magic_arm_config(config: CobotMagicArmConfig) -> None:
    """在连接硬件前尽早拦截明显错误的配置。"""

    if not config.model:
        raise ValueError("`model` must be a non-empty ARX robot model, e.g. 'X5'.")
    if not config.interface:
        raise ValueError("`interface` must be a non-empty CAN/EtherCAT interface name.")
    if config.controller_type != "joint_controller":
        raise ValueError("Cobot Magic currently supports only `controller_type='joint_controller'`.")
    if config.controller_dt is not None and config.controller_dt <= 0:
        raise ValueError("`controller_dt` must be > 0 when provided.")


def make_arx5_joint_controller(config: CobotMagicArmConfig) -> Any:
    """按 Piper 的方式在 Python 侧直接创建 ARX SDK 控制器。"""

    arx5 = get_arx5_sdk()

    # 先拿 SDK 的默认 robot/controller config，再覆盖 EvoRL 暴露出的安全相关字段。
    robot_config = arx5.RobotConfigFactory.get_instance().get_config(config.model)
    controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        config.controller_type,
        robot_config.joint_dof,
    )
    controller_config.background_send_recv = config.background_send_recv
    controller_config.gravity_compensation = config.gravity_compensation

    if hasattr(controller_config, "shutdown_to_passive"):
        controller_config.shutdown_to_passive = config.shutdown_to_passive
    if config.controller_dt is not None:
        controller_config.controller_dt = config.controller_dt

    controller = arx5.Arx5JointController(robot_config, controller_config, config.interface)
    set_arx5_log_level(controller, config.log_level, arx5)
    return controller


def set_arx5_log_level(controller: Any, log_level: str, arx5: ModuleType | None = None) -> None:
    """兼容 SDK 的 LogLevel 枚举，设置失败时让调用方看到明确错误。"""

    arx5 = arx5 or get_arx5_sdk()
    try:
        level = getattr(arx5.LogLevel, log_level.upper())
    except AttributeError as exc:
        raise ValueError(f"Unsupported ARX5 log level: {log_level}") from exc
    controller.set_log_level(level)


def arx5_state_to_observation(state: Any) -> RobotObservation:
    """把 SDK 的 JointState 展平成 LeRobot 标准的扁平 observation 字典。"""

    pos = np.asarray(state.pos(), dtype=np.float64)
    vel = np.asarray(state.vel(), dtype=np.float64)
    torque = np.asarray(state.torque(), dtype=np.float64)

    obs: RobotObservation = {}
    for idx, joint_name in enumerate(ARX5_JOINT_NAMES):
        obs[f"{joint_name}.pos"] = float(pos[idx])
        obs[f"{joint_name}.vel"] = float(vel[idx])
        obs[f"{joint_name}.torque"] = float(torque[idx])

    obs["gripper.pos"] = float(getattr(state, "gripper_pos", 0.0))
    obs["gripper.vel"] = float(getattr(state, "gripper_vel", 0.0))
    obs["gripper.torque"] = float(getattr(state, "gripper_torque", 0.0))
    return obs


def arx5_state_to_action(state: Any, *, include_gripper: bool = True) -> RobotAction:
    """把 SDK 状态转换成 teleop action；action 只包含位置目标。"""

    pos = np.asarray(state.pos(), dtype=np.float64)
    action: RobotAction = {
        f"{joint_name}.pos": float(pos[idx]) for idx, joint_name in enumerate(ARX5_JOINT_NAMES)
    }
    if include_gripper:
        # follower 发送命令时必须持有当前夹爪位置；否则会把 0.0 当成新的夹爪目标。
        action[ARX5_GRIPPER_KEY] = float(getattr(state, "gripper_pos", 0.0))
    return action


def make_arx5_joint_state(arx5: ModuleType, joint_dof: int, action: RobotAction) -> Any:
    """用扁平 action 构造 SDK 需要的 JointState 命令对象。"""

    cmd = arx5.JointState(joint_dof)
    cmd.pos()[:] = np.asarray([float(action[key]) for key in ARX5_JOINT_ACTION_KEYS], dtype=np.float64)
    cmd.gripper_pos = float(action.get(ARX5_GRIPPER_KEY, 0.0))
    return cmd
