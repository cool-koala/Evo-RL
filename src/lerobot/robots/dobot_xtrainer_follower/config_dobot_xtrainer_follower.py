#!/usr/bin/env python

from dataclasses import dataclass
from pathlib import Path

from ..config import RobotConfig


@RobotConfig.register_subclass("dobot_xtrainer_follower")
@dataclass
class DobotXTrainerFollowerConfig(RobotConfig):
    dobot_root: Path | None = None
    left_robot_ip: str = "192.168.5.1"
    right_robot_ip: str = "192.168.5.2"
    use_gripper: bool = True
    use_cameras: bool = True
    move_to_home_on_connect: bool = True
    action_mode: str = "joint"
    robot_command_hz: float = 30.0
    camera_fps: float = 30.0
    joint_interp_step_rad: float = 0.01
    joint_delta_limit_rad: float = 0.9
    ee_translation_limit_m: float = 0.05
    ee_rotation_limit_deg: float = 20.0
    ee_command_settle_s: float = 0.03
    indicator_lights_enabled: bool = True

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.action_mode not in {"joint", "ee_pose"}:
            raise ValueError("`action_mode` must be either 'joint' or 'ee_pose'.")
        if self.robot_command_hz <= 0:
            raise ValueError("`robot_command_hz` must be positive.")

