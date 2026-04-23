#!/usr/bin/env python

from dataclasses import dataclass
from pathlib import Path

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("dobot_xtrainer_leader")
@dataclass
class DobotXTrainerLeaderConfig(TeleoperatorConfig):
    dobot_root: Path | None = None
    manual_control: bool = True
    teleop_poll_hz: float = 60.0
    button_short_press_max_s: float = 0.5
    button_long_press_min_s: float = 1.0
    button_cooldown_s: float = 0.3
    leader_sync_max_joint_delta_rad: float = 0.12
    leader_sync_max_tcp_speed_mps: float = 1.0
    leader_sync_max_cartesian_step_m: float = 0.03
    leader_sync_max_rot_step_deg: float = 15.0
    leader_stall_min_progress_rad: float = 0.002
    leader_stall_max_cycles: int = 10
    leader_sensor_enabled: bool = True
    leader_protection_auto_release_torque: bool = True

    def __post_init__(self) -> None:
        if self.teleop_poll_hz <= 0:
            raise ValueError("`teleop_poll_hz` must be positive.")

