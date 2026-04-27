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

import configparser
import logging
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

LOGGER = logging.getLogger(__name__)

CAMERA_IMAGE_SHAPE = (480, 640, 3)

LEFT_JOINT_ACTION_KEYS = tuple(f"left_joint_{idx}.pos" for idx in range(1, 7)) + ("left_gripper.pos",)
RIGHT_JOINT_ACTION_KEYS = tuple(f"right_joint_{idx}.pos" for idx in range(1, 7)) + ("right_gripper.pos",)
JOINT_ACTION_KEYS = LEFT_JOINT_ACTION_KEYS + RIGHT_JOINT_ACTION_KEYS

LEFT_EE_ACTION_KEYS = (
    "left_ee.x",
    "left_ee.y",
    "left_ee.z",
    "left_ee.qx",
    "left_ee.qy",
    "left_ee.qz",
    "left_ee.qw",
    "left_gripper.pos",
)
RIGHT_EE_ACTION_KEYS = (
    "right_ee.x",
    "right_ee.y",
    "right_ee.z",
    "right_ee.qx",
    "right_ee.qy",
    "right_ee.qz",
    "right_ee.qw",
    "right_gripper.pos",
)
EE_ACTION_KEYS = LEFT_EE_ACTION_KEYS + RIGHT_EE_ACTION_KEYS

DOBOT_HOME_INTERMEDIATE = np.deg2rad(
    [-90, 30, -110, 20, 90, 90, 0, 90, -30, 110, -20, -90, -90, 0]
).astype(np.float64)
DOBOT_HOME_TARGET = np.deg2rad(
    [-90, 0, -90, 0, 90, 90, 0, 90, 0, 90, 0, -90, -90, 0]
).astype(np.float64)

CAMERA_LAYOUT = {
    "cam_high": {"ini_key": "top", "flip": True},
    "cam_left_wrist": {"ini_key": "left", "flip": False},
    "cam_right_wrist": {"ini_key": "right", "flip": True},
}


def default_dobot_root() -> Path:
    return Path(__file__).resolve().parents[4] / "dobot_xtrainer-master"


def ensure_dobot_import_path(dobot_root: str | Path | None = None) -> Path:
    root = Path(dobot_root).expanduser().resolve() if dobot_root is not None else default_dobot_root()
    if not root.exists():
        raise FileNotFoundError(
            f"Dobot Xtrainer workspace was not found at '{root}'. "
            "Set `dobot_root` to the repository root that contains `dobot_control/`."
        )
    root_str = str(root)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
    return root


def load_dobot_ini(dobot_root: str | Path | None = None) -> configparser.ConfigParser:
    root = ensure_dobot_import_path(dobot_root)
    ini_path = root / "scripts" / "dobot_config" / "dobot_settings.ini"
    if not ini_path.is_file():
        raise FileNotFoundError(f"Could not find Dobot settings file at '{ini_path}'.")

    parser = configparser.ConfigParser()
    parser.read(ini_path)
    return parser


def load_dobot_camera_ids(dobot_root: str | Path | None = None) -> dict[str, str]:
    parser = load_dobot_ini(dobot_root)
    return {name: parser.get("CAMERA", spec["ini_key"]) for name, spec in CAMERA_LAYOUT.items()}


def load_dobot_hand_configs(dobot_root: str | Path | None = None) -> dict[str, Any]:
    ensure_dobot_import_path(dobot_root)
    from scripts.manipulate_utils import load_ini_data_hands

    _, hand_configs = load_ini_data_hands()
    return hand_configs


def joint_action_dict_to_array(action: dict[str, Any], fallback: np.ndarray | None = None) -> np.ndarray:
    array = np.array(fallback, dtype=np.float64, copy=True) if fallback is not None else np.zeros(14, dtype=np.float64)
    for index, key in enumerate(JOINT_ACTION_KEYS):
        if key in action:
            array[index] = float(action[key])
    array[6] = float(np.clip(array[6], 0.0, 1.0))
    array[13] = float(np.clip(array[13], 0.0, 1.0))
    return array


def joint_array_to_dict(joints: np.ndarray) -> dict[str, float]:
    joints = np.asarray(joints, dtype=np.float64)
    if joints.shape[0] != 14:
        raise ValueError(f"Expected 14 joint values, got shape {joints.shape}.")
    return {key: float(joints[index]) for index, key in enumerate(JOINT_ACTION_KEYS)}


def ee_action_dict_to_array(action: dict[str, Any], fallback: np.ndarray | None = None) -> np.ndarray:
    array = np.array(fallback, dtype=np.float64, copy=True) if fallback is not None else np.zeros(16, dtype=np.float64)
    for index, key in enumerate(EE_ACTION_KEYS):
        if key in action:
            array[index] = float(action[key])
    array[7] = float(np.clip(array[7], 0.0, 1.0))
    array[15] = float(np.clip(array[15], 0.0, 1.0))
    return array


def ee_array_to_dict(ee_action: np.ndarray) -> dict[str, float]:
    ee_action = np.asarray(ee_action, dtype=np.float64)
    if ee_action.shape[0] != 16:
        raise ValueError(f"Expected 16 EE pose values, got shape {ee_action.shape}.")
    return {key: float(ee_action[index]) for index, key in enumerate(EE_ACTION_KEYS)}


def is_joint_action(action: dict[str, Any]) -> bool:
    return any(key in action for key in JOINT_ACTION_KEYS)


def is_ee_action(action: dict[str, Any]) -> bool:
    return any(key in action for key in EE_ACTION_KEYS)


def wrap_radians(angle: np.ndarray | float) -> np.ndarray | float:
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


def wrap_degrees(angle: np.ndarray | float) -> np.ndarray | float:
    return (np.asarray(angle) + 180.0) % 360.0 - 180.0


def ensure_finite_array(values: np.ndarray, *, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains NaN or infinite values.")
    return array


def normalize_quat_xyzw(
    quat_xyzw: np.ndarray | list[float] | tuple[float, float, float, float],
    *,
    fallback: np.ndarray | list[float] | tuple[float, float, float, float] | None = None,
    name: str = "quaternion",
) -> np.ndarray:
    quat = ensure_finite_array(np.asarray(quat_xyzw, dtype=np.float64), name=name)
    norm = float(np.linalg.norm(quat))
    if norm > 1e-6:
        return quat / norm

    if fallback is not None:
        fallback_quat = ensure_finite_array(np.asarray(fallback, dtype=np.float64), name=f"{name} fallback")
        fallback_norm = float(np.linalg.norm(fallback_quat))
        if fallback_norm > 1e-6:
            return fallback_quat / fallback_norm

    raise ValueError(f"{name} has near-zero norm and cannot be normalized.")


def euler_xyz_deg_to_quat_xyzw(euler_deg: np.ndarray | list[float] | tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = np.deg2rad(np.asarray(euler_deg, dtype=np.float64))
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        dtype=np.float64,
    )


def quat_xyzw_to_euler_xyz_deg(quat_xyzw: np.ndarray | list[float] | tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = normalize_quat_xyzw(quat_xyzw)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.rad2deg(np.array([roll, pitch, yaw], dtype=np.float64))


def mmdeg_pose_to_ee_array(pose_mmdeg: np.ndarray) -> np.ndarray:
    pose_mmdeg = np.asarray(pose_mmdeg, dtype=np.float64)
    if pose_mmdeg.shape[0] != 6:
        raise ValueError(f"Expected 6 pose values [x, y, z, rx, ry, rz], got {pose_mmdeg.shape}.")
    translation_m = pose_mmdeg[:3] / 1000.0
    quaternion = euler_xyz_deg_to_quat_xyzw(pose_mmdeg[3:6])
    return np.concatenate([translation_m, quaternion])


def bimanual_pose_mmdeg_to_ee_array(poses_mmdeg: np.ndarray, joint_state: np.ndarray | None = None) -> np.ndarray:
    poses_mmdeg = np.asarray(poses_mmdeg, dtype=np.float64)
    if poses_mmdeg.shape[0] != 12:
        raise ValueError(f"Expected 12 bimanual pose values, got {poses_mmdeg.shape}.")
    grippers = np.asarray(joint_state, dtype=np.float64)[[6, 13]] if joint_state is not None else np.zeros(2)
    left = np.concatenate([mmdeg_pose_to_ee_array(poses_mmdeg[:6]), [float(grippers[0])]])
    right = np.concatenate([mmdeg_pose_to_ee_array(poses_mmdeg[6:12]), [float(grippers[1])]])
    return np.concatenate([left, right])


def clamp_joint_delta(target: np.ndarray, current: np.ndarray, max_delta_rad: float) -> np.ndarray:
    target = ensure_finite_array(np.asarray(target, dtype=np.float64), name="joint target")
    current = ensure_finite_array(np.asarray(current, dtype=np.float64), name="joint current")
    delta = target - current
    joint_delta = wrap_radians(delta[:6])
    right_delta = wrap_radians(delta[7:13])
    delta[:6] = np.clip(joint_delta, -max_delta_rad, max_delta_rad)
    delta[7:13] = np.clip(right_delta, -max_delta_rad, max_delta_rad)
    delta[6] = np.clip(delta[6], -1.0, 1.0)
    delta[13] = np.clip(delta[13], -1.0, 1.0)
    result = current + delta
    result[6] = np.clip(result[6], 0.0, 1.0)
    result[13] = np.clip(result[13], 0.0, 1.0)
    return result


def joint_delta_vector(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    target = ensure_finite_array(np.asarray(target, dtype=np.float64), name="joint target")
    current = ensure_finite_array(np.asarray(current, dtype=np.float64), name="joint current")
    delta = target - current
    delta[:6] = wrap_radians(delta[:6])
    delta[7:13] = wrap_radians(delta[7:13])
    return delta


def clamp_cartesian_step(
    target: np.ndarray,
    current: np.ndarray,
    *,
    max_translation_m: float,
    max_rotation_deg: float,
) -> np.ndarray:
    target = ensure_finite_array(np.asarray(target, dtype=np.float64), name="cartesian target")
    current = ensure_finite_array(np.asarray(current, dtype=np.float64), name="cartesian current")
    result = np.array(target, dtype=np.float64, copy=True)
    current_quat = normalize_quat_xyzw(current[3:7], name="current cartesian quaternion")
    target_quat = normalize_quat_xyzw(target[3:7], fallback=current_quat, name="target cartesian quaternion")
    current = np.array(current, dtype=np.float64, copy=True)
    current[3:7] = current_quat
    result[3:7] = target_quat

    translation_delta = target[:3] - current[:3]
    translation_norm = float(np.linalg.norm(translation_delta))
    if translation_norm > max_translation_m > 0:
        result[:3] = current[:3] + translation_delta * (max_translation_m / translation_norm)

    current_euler = quat_xyzw_to_euler_xyz_deg(current[3:7])
    target_euler = quat_xyzw_to_euler_xyz_deg(result[3:7])
    delta_euler = wrap_degrees(target_euler - current_euler)
    limited_euler = current_euler + np.clip(delta_euler, -max_rotation_deg, max_rotation_deg)
    result[3:7] = euler_xyz_deg_to_quat_xyzw(limited_euler)
    result[7] = np.clip(result[7], 0.0, 1.0)
    return result


def set_indicator_lights(robot: Any, state: str) -> None:
    if robot is None:
        return

    if state == "off":
        robot.set_do_status([1, 0])
        robot.set_do_status([2, 0])
        robot.set_do_status([3, 0])
        return
    if state == "red":
        robot.set_do_status([3, 0])
        robot.set_do_status([2, 0])
        robot.set_do_status([1, 1])
        return
    if state == "yellow":
        robot.set_do_status([3, 0])
        robot.set_do_status([2, 1])
        robot.set_do_status([1, 0])
        return
    if state == "green":
        robot.set_do_status([3, 1])
        robot.set_do_status([2, 0])
        robot.set_do_status([1, 0])
        return
    raise ValueError(f"Unsupported Dobot indicator state: {state}")


def move_bimanual_joints_interpolated(
    robot: Any,
    target_joints: np.ndarray,
    *,
    step_size_rad: float,
    control_hz: float,
    active_arms: tuple[bool, bool] = (True, True),
    max_steps: int = 150,
) -> None:
    current = np.asarray(robot.get_joint_state(), dtype=np.float64)
    target = np.asarray(target_joints, dtype=np.float64)
    max_delta = float(np.max(np.abs(target - current)))
    if max_delta <= 1e-6:
        return

    steps = max(1, int(max_delta / max(step_size_rad, 1e-6)))
    steps = min(steps, max_steps)
    flags = np.array(active_arms, dtype=np.int64)
    sleep_s = 1.0 / max(control_hz, 1e-3)
    for waypoint in np.linspace(current, target, steps + 1, dtype=np.float64)[1:]:
        robot.command_joint_state(waypoint, flags)
        if sleep_s > 0:
            time.sleep(sleep_s)


def check_joint_safety(action: np.ndarray) -> tuple[bool, list[str]]:
    action = np.asarray(action, dtype=np.float64)
    warnings: list[str] = []
    if not (action[2] < 0):
        warnings.append("Left follower/leader joint_3 is outside the safe signed range.")
    if not (action[9] > 0):
        warnings.append("Right follower/leader joint_3 is outside the safe signed range.")
    return (len(warnings) == 0), warnings


def _dh_transformation_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)
    return np.array(
        [
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
            [0.0, sin_alpha, cos_alpha, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _claw_width(coef: float) -> float:
    claw_servo = 2.3818 - coef * 1.5401
    cos_claw_servo = np.cos(claw_servo)
    return 0.03 * cos_claw_servo + 0.5 * np.sqrt(0.0036 * cos_claw_servo**2 + 0.0028)


def _forward_kinematics(q0: float, q1: float, q2: float, q3: float, q4: float, q5: float, y: float) -> np.ndarray:
    dh_params = [
        (q0, 0.2234, 0.0, np.pi / 2),
        (q1 - np.pi / 2, 0.0, -0.280, 0.0),
        (q2, 0.0, -0.225, 0.0),
        (q3 - np.pi / 2, 0.1175, 0.0, np.pi / 2),
        (q4, 0.120, 0.0, -np.pi / 2),
        (q5, 0.088, 0.0, 0.0),
    ]

    transform = np.eye(4, dtype=np.float64)
    for params in dh_params:
        transform = transform @ _dh_transformation_matrix(*params)
    tool = np.eye(4, dtype=np.float64)
    tool[:3, 3] = np.array([0.0, y, 0.2], dtype=np.float64)
    return (transform @ tool)[:3, 3]


def calculate_tcp_positions_and_velocities(
    current_action: np.ndarray,
    last_action: np.ndarray,
    total_time_s: float,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    current_action = np.asarray(current_action, dtype=np.float64)
    last_action = np.asarray(last_action, dtype=np.float64)
    claw_left = _claw_width(current_action[6])
    claw_right = _claw_width(current_action[13])

    positions: dict[str, np.ndarray] = {}
    velocities: dict[str, np.ndarray] = {}

    for side in ("left", "right"):
        for jaw in ("left", "right"):
            coef = 1.0 if jaw == "left" else -1.0
            claw = claw_left if side == "left" else claw_right
            claw *= coef

            current_fk = _forward_kinematics(
                *(current_action[:6] if side == "left" else current_action[7:13]),
                claw,
            )
            last_fk = _forward_kinematics(
                *(last_action[:6] if side == "left" else last_action[7:13]),
                claw,
            )
            key = f"{side}_{jaw}"
            positions[key] = current_fk
            velocities[key] = (current_fk - last_fk) / max(total_time_s, 1e-6)

    return positions, velocities


def check_pose_protection(
    current_action: np.ndarray,
    last_action: np.ndarray,
    *,
    active_arms: tuple[bool, bool],
    total_time_s: float,
    max_tcp_speed_mps: float,
) -> tuple[bool, list[str]]:
    positions, velocities = calculate_tcp_positions_and_velocities(current_action, last_action, total_time_s)
    positions_mm = {key: value * 1000.0 for key, value in positions.items()}

    warnings: list[str] = []
    x_range_left = (-500.0, 400.0)
    x_range_right = (-400.0, 500.0)
    y_range = (-900.0, 0.0)
    z_range_left = 24.0
    z_range_right = 22.0

    def _within_zone(position: np.ndarray, x_range: tuple[float, float], z_min: float) -> bool:
        return (
            x_range[0] <= position[0] <= x_range[1]
            and y_range[0] <= position[1] <= y_range[1]
            and position[2] > z_min
        )

    if active_arms[0]:
        if velocities["left_left"][2] < -max_tcp_speed_mps or velocities["left_right"][2] < -max_tcp_speed_mps:
            warnings.append("Left leader TCP is moving downwards too fast.")
        for jaw in ("left_left", "left_right"):
            if not _within_zone(positions_mm[jaw], x_range_left, z_range_left):
                warnings.append("Left leader moved outside the configured safe workspace.")
                break

    if active_arms[1]:
        if velocities["right_left"][2] < -max_tcp_speed_mps or velocities["right_right"][2] < -max_tcp_speed_mps:
            warnings.append("Right leader TCP is moving downwards too fast.")
        for jaw in ("right_left", "right_right"):
            if not _within_zone(positions_mm[jaw], x_range_right, z_range_right):
                warnings.append("Right leader moved outside the configured safe workspace.")
                break

    return (len(warnings) == 0), warnings


def convert_leader_command_to_driver_space(leader_robot: Any, command: np.ndarray) -> np.ndarray:
    command = np.asarray(command, dtype=np.float64)
    converted = np.array(command, dtype=np.float64, copy=True)
    gripper_open_close = getattr(leader_robot, "gripper_open_close", None)
    if gripper_open_close is not None:
        converted[-1] = gripper_open_close[0] + converted[-1] * (gripper_open_close[1] - gripper_open_close[0])
    return converted


class LatestCameraFrame:
    def __init__(self, camera: Any, *, name: str, target_fps: float | None = None):
        self.camera = camera
        self.name = name
        self.target_fps = float(target_fps) if target_fps is not None and target_fps > 0 else 0.0
        self._frame = np.zeros(CAMERA_IMAGE_SHAPE, dtype=np.uint8)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name=f"{self.name}-camera-reader", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        frame_period_s = 1.0 / self.target_fps if self.target_fps > 0 else 0.0
        while not self._stop_event.is_set():
            started_t = time.perf_counter()
            try:
                frame, _ = self.camera.read()
                with self._lock:
                    self._frame = np.array(frame, copy=True)
            except Exception:
                LOGGER.exception("Failed to read frame from Dobot camera '%s'.", self.name)
                time.sleep(0.1)
                continue

            if frame_period_s > 0:
                remaining_s = frame_period_s - (time.perf_counter() - started_t)
                if remaining_s > 0:
                    self._stop_event.wait(remaining_s)

    def get(self) -> np.ndarray:
        with self._lock:
            return np.array(self._frame, copy=True)

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        pipeline = getattr(self.camera, "_pipeline", None)
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                LOGGER.debug("Failed to stop RealSense pipeline for %s.", self.name, exc_info=True)


@dataclass
class ButtonPressTracker:
    short_press_max_s: float
    long_press_min_s: float
    cooldown_s: float

    def __post_init__(self) -> None:
        self._last_values = np.zeros((2, 3), dtype=np.int64)
        self._press_start = np.full((2, 2), np.nan, dtype=np.float64)
        self._last_emit_time = 0.0

    def update(self, key_state: np.ndarray) -> dict[str, bool]:
        now = time.monotonic()
        events = {
            "toggle_intervention": False,
            "sync_request": False,
            "end_episode": False,
        }
        key_state = np.asarray(key_state, dtype=np.int64)
        key_delta = key_state - self._last_values

        for side in range(2):
            for button in range(2):
                if key_delta[side, button] < 0:
                    self._press_start[side, button] = now
                elif key_delta[side, button] > 0 and not np.isnan(self._press_start[side, button]):
                    press_duration = now - float(self._press_start[side, button])
                    self._press_start[side, button] = np.nan
                    if now - self._last_emit_time < self.cooldown_s:
                        continue
                    self._last_emit_time = now
                    if button == 0 and press_duration <= self.short_press_max_s:
                        events["toggle_intervention"] = True
                    elif button == 0 and press_duration >= self.long_press_min_s:
                        events["sync_request"] = True
                    elif button == 1 and press_duration <= self.short_press_max_s:
                        events["end_episode"] = True

        self._last_values = key_state
        return events
