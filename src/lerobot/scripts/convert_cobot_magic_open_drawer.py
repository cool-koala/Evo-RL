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

"""Convert local Cobot Magic/ARX-X5 open-drawer HDF5 data into the HIL training schema."""

from __future__ import annotations

import argparse
import json
import logging
import shutil
import subprocess
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset

LOGGER = logging.getLogger(__name__)

DEFAULT_LOCAL_ARCHIVE = Path(
    "/media/abc/\u65b0\u52a0\u5377/ARX5_Data/Data_30fps/open_the_drawer/open_the_drawer_3_20_500.tar.zst"
)
DEFAULT_WORK_DIR = Path("data/cobot_magic_open_drawer_local_bimanual_v1")
DEFAULT_DATASET_ROOT = DEFAULT_WORK_DIR / "lerobot"
DEFAULT_REPO_ID = "local/cobot_magic_open_drawer_local_bimanual_v1"
DEFAULT_TASK = "open the drawer"
DEFAULT_FPS = 30

CAMERA_KEY = "observation.images.cam_left_wrist"
LOCAL_CAMERA_KEY = "cam_left_wrist"
IMAGE_SHAPE = (480, 640, 3)
ACTION_NAMES = [
    *(f"left_joint_{idx}.pos" for idx in range(1, 7)),
    "left_gripper.pos",
    *(f"right_joint_{idx}.pos" for idx in range(1, 7)),
    "right_gripper.pos",
]
STATE_NAMES = [
    *(
        f"{side}_{joint}.{suffix}"
        for side in ("left", "right")
        for joint in [*(f"joint_{idx}" for idx in range(1, 7)), "gripper"]
        for suffix in ("pos", "vel", "torque")
    )
]
JOINT_ACTION_INDICES = [*range(0, 6), *range(7, 13)]
GRIPPER_ACTION_INDICES = [6, 13]
REQUIRED_HDF5_KEYS = (
    "action",
    "observations/qpos",
    f"observations/images/{LOCAL_CAMERA_KEY}",
)


@dataclass(frozen=True)
class LocalEpisodeSpec:
    path: str
    frames: int
    first_qpos: list[float]


@dataclass(frozen=True)
class SkippedEpisode:
    path: str
    reason: str


def make_cobot_magic_left_wrist_features(*, use_videos: bool = True) -> dict[str, dict[str, Any]]:
    return {
        "action": {
            "dtype": "float32",
            "shape": (len(ACTION_NAMES),),
            "names": ACTION_NAMES,
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (len(STATE_NAMES),),
            "names": STATE_NAMES,
        },
        CAMERA_KEY: {
            "dtype": "video" if use_videos else "image",
            "shape": IMAGE_SHAPE,
            "names": ["height", "width", "channels"],
        },
    }


def build_action_14(action: Any) -> np.ndarray:
    return _as_vector(action, min_size=14)[:14].astype(np.float32)


def build_state_42(
    qpos: Any,
    *,
    qvel: Any | None = None,
    effort: Any | None = None,
) -> np.ndarray:
    pos = _as_vector(qpos, min_size=14)[:14]
    vel = _as_optional_vector(qvel, size=14)
    torque = _as_optional_vector(effort, size=14)

    state: list[float] = []
    for side_start in (0, 7):
        for idx in range(7):
            source_idx = side_start + idx
            state.extend(
                [
                    float(pos[source_idx]),
                    float(vel[source_idx]),
                    float(torque[source_idx]),
                ]
            )
    return np.asarray(state, dtype=np.float32)


def build_joint_pose_14(joint_pos: Any) -> dict[str, float]:
    values = _as_vector(joint_pos, min_size=14)[:14]
    return {name: float(value) for name, value in zip(ACTION_NAMES, values, strict=True)}


def build_reset_pose_payload(
    reset_qpos: Any,
    *,
    source_episode_count: int,
    task: str,
) -> dict[str, Any]:
    return {
        "captured_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "description": "Cobot Magic/ARX-X5 open-drawer local HDF5 median start pose.",
        "joint_pos": build_joint_pose_14(reset_qpos),
        "robot_type": "cobot_magic_ros_follower",
        "source": {
            "episode_count": int(source_episode_count),
            "statistic": "median_first_qpos",
            "task": task,
        },
        "units": "rad",
    }


def resize_rgb_image(image: Any, shape: tuple[int, int, int] = IMAGE_SHAPE) -> np.ndarray:
    arr = _image_to_numpy(image)
    height, width, channels = shape
    if channels != 3:
        raise ValueError(f"Only RGB output is supported, got {channels} channels.")
    if arr.shape[:2] != (height, width):
        arr = cv2.resize(arr, (width, height), interpolation=cv2.INTER_LINEAR)
    return np.ascontiguousarray(arr, dtype=np.uint8)


def convert_local_hdf5_episode(
    hdf5_path: Path,
    dataset: LeRobotDataset,
    *,
    task: str,
) -> dict[str, Any]:
    h5py = _import_h5py()

    with h5py.File(hdf5_path, "r") as episode:
        qpos = np.asarray(episode["observations/qpos"], dtype=np.float32)
        action = np.asarray(episode["action"], dtype=np.float32)
        qvel = (
            np.asarray(episode["observations/qvel"], dtype=np.float32)
            if "observations/qvel" in episode
            else None
        )
        effort = (
            np.asarray(episode["observations/effort"], dtype=np.float32)
            if "observations/effort" in episode
            else None
        )
        images = episode[f"observations/images/{LOCAL_CAMERA_KEY}"]
        episode_len = min(len(qpos), len(action), len(images))
        episode_report = _new_episode_report()

        for idx in range(episode_len):
            converted_action = build_action_14(action[idx])
            converted_state = build_state_42(
                qpos[idx],
                qvel=None if qvel is None else qvel[idx],
                effort=None if effort is None else effort[idx],
            )
            _update_episode_report(episode_report, action=converted_action, state=converted_state)
            dataset.add_frame(
                {
                    "action": converted_action,
                    "observation.state": converted_state,
                    CAMERA_KEY: resize_rgb_image(images[idx]),
                    "task": task,
                }
            )
        dataset.save_episode()
    return episode_report


def extract_local_archive_if_needed(
    archive_path: Path,
    extract_dir: Path,
    *,
    include_fail: bool,
) -> list[Path]:
    existing = _find_hdf5_episodes(extract_dir, include_fail=include_fail)
    if existing:
        LOGGER.info("Using %d already extracted local HDF5 episodes under %s.", len(existing), extract_dir)
        return existing

    extract_dir.mkdir(parents=True, exist_ok=True)
    cmd = ["tar"]
    if not include_fail:
        cmd.append("--exclude=*/Fail/*")
    cmd.extend(["--zstd", "-xf", str(archive_path), "-C", str(extract_dir)])
    LOGGER.info("Extracting local archive %s to %s.", archive_path, extract_dir)
    subprocess.run(cmd, check=True)
    episodes = _find_hdf5_episodes(extract_dir, include_fail=include_fail)
    if not episodes:
        raise FileNotFoundError(f"No HDF5 episodes were extracted from {archive_path}.")
    return episodes


def resolve_local_hdf5_episodes(
    source_path: Path,
    work_dir: Path,
    *,
    include_fail: bool,
) -> list[Path]:
    if not source_path.exists():
        raise FileNotFoundError(f"Local source path does not exist: {source_path}")
    if source_path.is_dir():
        episodes = _find_hdf5_episodes(source_path, include_fail=include_fail)
    else:
        episodes = extract_local_archive_if_needed(
            source_path,
            work_dir / "source_local_hdf5",
            include_fail=include_fail,
        )
    if not episodes:
        raise FileNotFoundError(f"No local HDF5 episodes found under {source_path}.")
    return episodes


def inspect_local_hdf5_episode(
    hdf5_path: Path,
    *,
    min_episode_frames: int,
    max_action_delta_rad: float | None,
    max_gripper_delta: float | None,
) -> LocalEpisodeSpec | SkippedEpisode:
    h5py = _import_h5py()

    try:
        with h5py.File(hdf5_path, "r") as episode:
            for key in REQUIRED_HDF5_KEYS:
                if key not in episode:
                    return SkippedEpisode(str(hdf5_path), f"missing required key {key!r}")

            qpos_ds = episode["observations/qpos"]
            action_ds = episode["action"]
            image_ds = episode[f"observations/images/{LOCAL_CAMERA_KEY}"]
            if qpos_ds.ndim != 2 or qpos_ds.shape[1] < 14:
                return SkippedEpisode(str(hdf5_path), f"invalid qpos shape {qpos_ds.shape}")
            if action_ds.ndim != 2 or action_ds.shape[1] < 14:
                return SkippedEpisode(str(hdf5_path), f"invalid action shape {action_ds.shape}")
            if image_ds.ndim != 4 or image_ds.shape[-1] != 3:
                return SkippedEpisode(str(hdf5_path), f"invalid left wrist image shape {image_ds.shape}")

            episode_len = min(len(qpos_ds), len(action_ds), len(image_ds))
            if episode_len < min_episode_frames:
                return SkippedEpisode(str(hdf5_path), f"too short ({episode_len} frames)")

            qpos = np.asarray(qpos_ds[:episode_len, :14], dtype=np.float32)
            action = np.asarray(action_ds[:episode_len, :14], dtype=np.float32)
            if not np.isfinite(qpos).all():
                return SkippedEpisode(str(hdf5_path), "qpos contains NaN or Inf")
            if not np.isfinite(action).all():
                return SkippedEpisode(str(hdf5_path), "action contains NaN or Inf")
            for key in ("observations/qvel", "observations/effort"):
                if key in episode:
                    values = np.asarray(episode[key][:episode_len, :14], dtype=np.float32)
                    if not np.isfinite(values).all():
                        return SkippedEpisode(str(hdf5_path), f"{key} contains NaN or Inf")

            if episode_len > 1:
                abs_delta = np.abs(np.diff(action, axis=0))
                if max_action_delta_rad is not None:
                    max_joint_delta = float(abs_delta[:, JOINT_ACTION_INDICES].max())
                    if max_joint_delta > max_action_delta_rad:
                        return SkippedEpisode(
                            str(hdf5_path),
                            f"joint action jump {max_joint_delta:.3f} > {max_action_delta_rad:.3f}",
                        )
                if max_gripper_delta is not None:
                    max_gripper_jump = float(abs_delta[:, GRIPPER_ACTION_INDICES].max())
                    if max_gripper_jump > max_gripper_delta:
                        return SkippedEpisode(
                            str(hdf5_path),
                            f"gripper action jump {max_gripper_jump:.3f} > {max_gripper_delta:.3f}",
                        )

            return LocalEpisodeSpec(
                path=str(hdf5_path),
                frames=int(episode_len),
                first_qpos=qpos[0].astype(np.float64).tolist(),
            )
    except OSError as exc:
        return SkippedEpisode(str(hdf5_path), f"could not read HDF5 file: {exc}")


def collect_valid_local_episode_specs(
    hdf5_paths: list[Path],
    *,
    min_episode_frames: int,
    max_action_delta_rad: float | None,
    max_gripper_delta: float | None,
    max_start_pose_deviation: float | None,
) -> tuple[list[LocalEpisodeSpec], list[SkippedEpisode], np.ndarray]:
    specs: list[LocalEpisodeSpec] = []
    skipped: list[SkippedEpisode] = []
    for path in hdf5_paths:
        result = inspect_local_hdf5_episode(
            path,
            min_episode_frames=min_episode_frames,
            max_action_delta_rad=max_action_delta_rad,
            max_gripper_delta=max_gripper_delta,
        )
        if isinstance(result, SkippedEpisode):
            skipped.append(result)
        else:
            specs.append(result)

    if not specs:
        raise RuntimeError("No valid local HDF5 episodes remain after basic validation.")

    first_qpos = np.asarray([spec.first_qpos for spec in specs], dtype=np.float32)
    reset_qpos = np.median(first_qpos, axis=0).astype(np.float32)

    if max_start_pose_deviation is not None:
        filtered_specs: list[LocalEpisodeSpec] = []
        for spec, qpos in zip(specs, first_qpos, strict=True):
            max_delta = float(np.max(np.abs(qpos - reset_qpos)))
            if max_delta > max_start_pose_deviation:
                skipped.append(
                    SkippedEpisode(
                        spec.path,
                        f"start pose delta {max_delta:.3f} > {max_start_pose_deviation:.3f}",
                    )
                )
            else:
                filtered_specs.append(spec)
        specs = filtered_specs

    if not specs:
        raise RuntimeError("No valid local HDF5 episodes remain after start-pose validation.")

    reset_qpos = np.median(np.asarray([spec.first_qpos for spec in specs], dtype=np.float32), axis=0)
    return specs, skipped, reset_qpos.astype(np.float32)


def write_reset_pose(reset_pose_path: Path, payload: dict[str, Any], *, overwrite: bool) -> None:
    if reset_pose_path.exists() and not overwrite:
        raise FileExistsError(f"Reset pose file already exists: {reset_pose_path}. Pass --overwrite.")
    reset_pose_path.parent.mkdir(parents=True, exist_ok=True)
    reset_pose_path.write_text(json.dumps(payload, indent=2, sort_keys=True))
    LOGGER.info("Wrote local median reset pose to %s.", reset_pose_path)


def convert(args: argparse.Namespace) -> dict[str, Any]:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    work_dir = args.work_dir
    dataset_root = args.dataset_root
    reset_pose_path = args.reset_pose_output or (work_dir / "reset_pose.json")

    if args.overwrite and dataset_root.exists():
        LOGGER.info("Removing existing dataset root %s.", dataset_root)
        shutil.rmtree(dataset_root)
    if dataset_root.exists():
        raise FileExistsError(f"Dataset root already exists: {dataset_root}. Pass --overwrite to replace it.")

    work_dir.mkdir(parents=True, exist_ok=True)
    local_paths = resolve_local_hdf5_episodes(
        args.local_source,
        work_dir,
        include_fail=args.include_local_fail,
    )
    if args.max_local_episodes is not None:
        local_paths = local_paths[: args.max_local_episodes]

    specs, skipped, reset_qpos = collect_valid_local_episode_specs(
        local_paths,
        min_episode_frames=args.min_episode_frames,
        max_action_delta_rad=args.max_action_delta_rad,
        max_gripper_delta=args.max_gripper_delta,
        max_start_pose_deviation=args.max_start_pose_deviation,
    )
    reset_payload = build_reset_pose_payload(reset_qpos, source_episode_count=len(specs), task=args.task)
    write_reset_pose(reset_pose_path, reset_payload, overwrite=args.overwrite)

    dataset = LeRobotDataset.create(
        args.repo_id,
        args.fps,
        root=dataset_root,
        robot_type="cobot_magic_ros_follower",
        features=make_cobot_magic_left_wrist_features(use_videos=True),
        use_videos=True,
        image_writer_threads=args.image_writer_threads,
        vcodec=args.vcodec,
    )

    report: dict[str, Any] = {
        "repo_id": args.repo_id,
        "dataset_root": str(dataset_root),
        "reset_pose_path": str(reset_pose_path),
        "fps": args.fps,
        "image_key": CAMERA_KEY,
        "image_shape": list(IMAGE_SHAPE),
        "action_names": ACTION_NAMES,
        "state_names": STATE_NAMES,
        "reset_joint_pos": reset_payload["joint_pos"],
        "filters": {
            "min_episode_frames": args.min_episode_frames,
            "max_action_delta_rad": args.max_action_delta_rad,
            "max_gripper_delta": args.max_gripper_delta,
            "max_start_pose_deviation": args.max_start_pose_deviation,
        },
        "sources": {},
        "skipped_episodes_count": len(skipped),
        "skipped_episodes": [asdict(item) for item in skipped[: max(args.max_skipped_report_entries, 0)]],
    }

    try:
        local_report = _new_source_report()
        for episode_count, spec in enumerate(specs, start=1):
            episode_report = convert_local_hdf5_episode(Path(spec.path), dataset, task=args.task)
            _merge_episode_report(local_report, episode_report)
            if episode_count % args.log_every == 0:
                LOGGER.info("Converted %d local episodes.", episode_count)
        local_report["source_episode_count"] = len(local_paths)
        local_report["valid_episode_count"] = len(specs)
        local_report["skipped_episode_count"] = len(skipped)
        local_report["start_pose_median"] = reset_qpos.astype(np.float64).tolist()
        report["sources"]["local_hdf5"] = local_report
    finally:
        dataset.finalize()

    report_path = work_dir / "conversion_report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True))
    LOGGER.info("Wrote conversion report to %s.", report_path)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", default=DEFAULT_REPO_ID)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--work-dir", type=Path, default=DEFAULT_WORK_DIR)
    parser.add_argument("--reset-pose-output", type=Path)
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--vcodec", default="h264", choices=("h264", "hevc", "libsvtav1"))
    parser.add_argument("--image-writer-threads", type=int, default=4)
    parser.add_argument("--log-every", type=int, default=10)

    parser.add_argument("--local-source", type=Path, default=DEFAULT_LOCAL_ARCHIVE)
    parser.add_argument("--include-local-fail", action="store_true")
    parser.add_argument("--max-local-episodes", type=int)

    parser.add_argument("--min-episode-frames", type=int, default=30)
    parser.add_argument("--max-action-delta-rad", type=float, default=1.0)
    parser.add_argument("--max-gripper-delta", type=float, default=1.5)
    parser.add_argument("--max-start-pose-deviation", type=float, default=2.0)
    parser.add_argument("--max-skipped-report-entries", type=int, default=200)

    # Accepted for old shell snippets. The converter is local-only by default now.
    parser.add_argument("--skip-hf", action="store_true", help=argparse.SUPPRESS)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    convert(args)


def _import_h5py():
    try:
        import h5py
    except ImportError as exc:
        raise ImportError(
            "Converting local Cobot Magic HDF5 data requires h5py. "
            'Install it with `pip install -e ".[cobot_magic]"`.'
        ) from exc
    return h5py


def _as_vector(value: Any, *, min_size: int) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    arr = np.asarray(value, dtype=np.float32).reshape(-1)
    if arr.size < min_size:
        raise ValueError(f"Expected at least {min_size} values, got shape {arr.shape}.")
    return arr


def _as_optional_vector(value: Any | None, *, size: int) -> np.ndarray:
    if value is None:
        return np.zeros(size, dtype=np.float32)
    return _as_vector(value, min_size=size)[:size]


def _image_to_numpy(image: Any) -> np.ndarray:
    if hasattr(image, "detach"):
        image = image.detach().cpu().numpy()
    arr = np.asarray(image)
    if arr.ndim != 3:
        raise ValueError(f"Expected an RGB image with 3 dimensions, got shape {arr.shape}.")
    if arr.shape[0] in {1, 3, 4} and arr.shape[-1] not in {1, 3}:
        arr = np.moveaxis(arr, 0, -1)
    if arr.shape[-1] == 4:
        arr = arr[..., :3]
    if arr.shape[-1] == 1:
        arr = np.repeat(arr, 3, axis=-1)
    if arr.shape[-1] != 3:
        raise ValueError(f"Expected an RGB image, got shape {arr.shape}.")
    if np.issubdtype(arr.dtype, np.floating):
        if np.nanmax(arr) <= 1.0:
            arr = arr * 255.0
        arr = np.clip(arr, 0.0, 255.0)
    return arr.astype(np.uint8, copy=False)


def _find_hdf5_episodes(root: Path, *, include_fail: bool) -> list[Path]:
    paths = sorted(root.rglob("*.hdf5"))
    if include_fail:
        return paths
    return [path for path in paths if "Fail" not in path.parts]


def _new_source_report() -> dict[str, Any]:
    return {
        "episodes": 0,
        "frames": 0,
        "action_min": None,
        "action_max": None,
        "state_min": None,
        "state_max": None,
        "left_gripper_min": None,
        "left_gripper_max": None,
        "right_gripper_min": None,
        "right_gripper_max": None,
    }


def _new_episode_report() -> dict[str, Any]:
    return {
        "frames": 0,
        "action_min": None,
        "action_max": None,
        "state_min": None,
        "state_max": None,
    }


def _update_episode_report(report: dict[str, Any], *, action: np.ndarray, state: np.ndarray) -> None:
    report["frames"] += 1
    _update_min_max(report, "action", action)
    _update_min_max(report, "state", state)


def _merge_episode_report(report: dict[str, Any], episode_report: dict[str, Any]) -> None:
    report["episodes"] += 1
    report["frames"] += int(episode_report["frames"])
    for key in ("action", "state"):
        min_key = f"{key}_min"
        max_key = f"{key}_max"
        if episode_report[min_key] is None:
            continue
        _merge_min_max(report, min_key, max_key, episode_report[min_key], episode_report[max_key])
    if report["action_min"] is not None:
        report["left_gripper_min"] = report["action_min"][6]
        report["left_gripper_max"] = report["action_max"][6]
        report["right_gripper_min"] = report["action_min"][13]
        report["right_gripper_max"] = report["action_max"][13]


def _update_min_max(report: dict[str, Any], key: str, values: np.ndarray) -> None:
    min_key = f"{key}_min"
    max_key = f"{key}_max"
    value_min = values.astype(np.float64).tolist()
    value_max = values.astype(np.float64).tolist()
    _merge_min_max(report, min_key, max_key, value_min, value_max)


def _merge_min_max(
    report: dict[str, Any],
    min_key: str,
    max_key: str,
    value_min: list[float],
    value_max: list[float],
) -> None:
    if report[min_key] is None:
        report[min_key] = value_min
        report[max_key] = value_max
        return
    report[min_key] = [min(old, new) for old, new in zip(report[min_key], value_min, strict=True)]
    report[max_key] = [max(old, new) for old, new in zip(report[max_key], value_max, strict=True)]


if __name__ == "__main__":
    main()
