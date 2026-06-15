#!/usr/bin/env python
"""Run Cobot Magic HIL against a remote openpi joint policy server.

The joint server is the executable control path. EE-pose policy inference is
disabled by default and can be queried as shadow-only debug data with
--enable-ee-shadow, but it is never sent to the robot.
"""

from __future__ import annotations

import argparse
import logging
import shutil
import sys
import time
from collections.abc import Sequence
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
OPENPI_CLIENT_SRC = REPO_ROOT / "openpi" / "packages" / "openpi-client" / "src"
for path in (str(SRC_ROOT), str(OPENPI_CLIENT_SRC)):
    if path not in sys.path:
        sys.path.insert(0, path)

from lerobot.utils.recording_annotations import resolve_collector_policy_id, resolve_episode_success_label


ACTION = "action"
OBS_STR = "observation"
INTERVENTION_STATE_POLICY = 0.0
INTERVENTION_STATE_ACTIVE = 1.0


DEFAULT_HOST = "115.190.52.37"
DEFAULT_JOINT_PORT = 5352
DEFAULT_EE_PORT = 5353

JOINT_ACTION_NAMES = [
    "left_joint_1.pos",
    "left_joint_2.pos",
    "left_joint_3.pos",
    "left_joint_4.pos",
    "left_joint_5.pos",
    "left_joint_6.pos",
    "left_gripper.pos",
    "right_joint_1.pos",
    "right_joint_2.pos",
    "right_joint_3.pos",
    "right_joint_4.pos",
    "right_joint_5.pos",
    "right_joint_6.pos",
    "right_gripper.pos",
]
OPENPI_SWAPPED_JOINT_ACTION_NAMES = [
    *JOINT_ACTION_NAMES[7:14],
    *JOINT_ACTION_NAMES[0:7],
]
EE_ACTION_NAMES = [
    "left_ee.x",
    "left_ee.y",
    "left_ee.z",
    "left_ee.wx",
    "left_ee.wy",
    "left_ee.wz",
    "left_ee.gripper_pos",
    "right_ee.x",
    "right_ee.y",
    "right_ee.z",
    "right_ee.wx",
    "right_ee.wy",
    "right_ee.wz",
    "right_ee.gripper_pos",
]
ACTION_NAMES = [*JOINT_ACTION_NAMES, *EE_ACTION_NAMES]
STATE_NAMES = [
    "left_joint_1.pos",
    "left_joint_1.vel",
    "left_joint_1.torque",
    "left_joint_2.pos",
    "left_joint_2.vel",
    "left_joint_2.torque",
    "left_joint_3.pos",
    "left_joint_3.vel",
    "left_joint_3.torque",
    "left_joint_4.pos",
    "left_joint_4.vel",
    "left_joint_4.torque",
    "left_joint_5.pos",
    "left_joint_5.vel",
    "left_joint_5.torque",
    "left_joint_6.pos",
    "left_joint_6.vel",
    "left_joint_6.torque",
    "left_gripper.pos",
    "left_gripper.vel",
    "left_gripper.torque",
    "right_joint_1.pos",
    "right_joint_1.vel",
    "right_joint_1.torque",
    "right_joint_2.pos",
    "right_joint_2.vel",
    "right_joint_2.torque",
    "right_joint_3.pos",
    "right_joint_3.vel",
    "right_joint_3.torque",
    "right_joint_4.pos",
    "right_joint_4.vel",
    "right_joint_4.torque",
    "right_joint_5.pos",
    "right_joint_5.vel",
    "right_joint_5.torque",
    "right_joint_6.pos",
    "right_joint_6.vel",
    "right_joint_6.torque",
    "right_gripper.pos",
    "right_gripper.vel",
    "right_gripper.torque",
    *EE_ACTION_NAMES,
]
CAMERA_NAMES = ("cam_high", "cam_left_wrist", "cam_right_wrist")
COLLECTOR_POLICY_ID_NAMES = ("collector_policy_id",)


@dataclass
class PolicyStep:
    action: dict[str, float]
    raw_vector: np.ndarray
    source: str
    chunk_id: int
    chunk_step: int
    chunk_len: int


class _NoopListener:
    def stop(self) -> None:
        return


class TTYKeyboardListener:
    """Minimal non-blocking keyboard controls for headless SSH sessions."""

    def __init__(
        self,
        events: dict[str, Any],
        *,
        intervention_key: str,
        success_key: str,
        failure_key: str,
    ) -> None:
        import os
        import select
        import termios
        import threading
        import tty

        self._os = os
        self._select = select
        self._termios = termios
        self._tty = tty
        self._threading = threading
        self.events = events
        self.intervention_key = intervention_key.lower()
        self.success_key = success_key.lower()
        self.failure_key = failure_key.lower()
        self._fd = sys.stdin.fileno()
        self._old_attrs = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_toggle = 0.0

    def start(self) -> None:
        self._old_attrs = self._termios.tcgetattr(self._fd)
        self._tty.setcbreak(self._fd)
        self._thread = self._threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self._old_attrs is not None:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old_attrs)
            self._old_attrs = None

    def _run(self) -> None:
        while not self._stop.is_set():
            ready, _, _ = self._select.select([self._fd], [], [], 0.1)
            if not ready:
                continue
            chunk = self._os.read(self._fd, 1)
            if not chunk:
                continue
            if chunk == b"\x1b":
                self.events["stop_recording"] = True
                self.events["exit_early"] = True
                continue
            key = chunk.decode("utf-8", errors="ignore").lower()
            if key == self.intervention_key:
                now = time.monotonic()
                if now - self._last_toggle >= 0.5:
                    self._last_toggle = now
                    self.events["toggle_intervention"] = True
            elif key == self.success_key:
                self.events["episode_outcome"] = "success"
                self.events["exit_early"] = True
            elif key == self.failure_key:
                self.events["episode_outcome"] = "failure"
                self.events["exit_early"] = True


def init_events_and_keyboard(
    *,
    intervention_key: str,
    success_key: str,
    failure_key: str,
) -> tuple[Any, dict[str, Any]]:
    events = {
        "exit_early": False,
        "rerecord_episode": False,
        "stop_recording": False,
        "toggle_intervention": False,
        "episode_outcome": None,
    }
    if not sys.stdin.isatty():
        logging.warning("No interactive TTY; keyboard HIL controls are disabled.")
        return _NoopListener(), events

    listener = TTYKeyboardListener(
        events,
        intervention_key=intervention_key,
        success_key=success_key,
        failure_key=failure_key,
    )
    listener.start()
    return listener, events


def precise_sleep(seconds: float) -> None:
    if seconds > 0:
        time.sleep(seconds)


def build_dataset_frame_local(
    ds_features: dict[str, dict[str, Any]],
    values: dict[str, Any],
    *,
    prefix: str,
) -> dict[str, np.ndarray]:
    frame: dict[str, np.ndarray] = {}
    default_features = {"timestamp", "frame_index", "episode_index", "index", "task_index"}
    for key, ft in ds_features.items():
        if key in default_features or not key.startswith(prefix):
            continue
        if ft["dtype"] == "float32" and len(ft["shape"]) == 1:
            frame[key] = np.asarray([values[name] for name in ft["names"]], dtype=np.float32)
        elif ft["dtype"] in {"image", "video"}:
            frame[key] = values[key.removeprefix(f"{prefix}.images.")]
    return frame


def openpi_client_import_error(exc: ImportError) -> ImportError:
    return ImportError(
        "openpi-client dependencies are missing. Install them in this environment with "
        f"`pip install -e openpi/packages/openpi-client`. Original error: {exc}"
    )


def make_openpi_uri(host: str, port: int | None) -> str:
    if host.startswith("ws"):
        return host
    if ":" in host.rsplit("/", 1)[-1]:
        return f"ws://{host}"
    if port is None:
        return f"ws://{host}"
    return f"ws://{host}:{port}"


class FiniteOpenPiClientPolicy:
    """Thin timeout wrapper around openpi_client.WebsocketClientPolicy."""

    def __init__(
        self,
        *,
        host: str,
        port: int | None,
        connect_timeout_s: float,
        request_timeout_s: float,
        api_key: str | None = None,
        name: str = "policy",
    ) -> None:
        self.host = make_openpi_uri(host, port)
        self.connect_timeout_s = connect_timeout_s
        self.request_timeout_s = request_timeout_s
        self.api_key = api_key
        self.name = name
        self._policy: Any | None = None
        self.metadata: dict[str, Any] = {}

    def connect(self) -> None:
        try:
            from openpi_client import websocket_client_policy
        except ImportError as exc:
            raise openpi_client_import_error(exc) from exc

        connect_timeout_s = self.connect_timeout_s
        request_timeout_s = self.request_timeout_s
        name = self.name

        class _FiniteWebsocketClientPolicy(websocket_client_policy.WebsocketClientPolicy):
            def _recv_with_timeout(self, conn: Any) -> bytes:
                try:
                    response = conn.recv(timeout=request_timeout_s)
                except TypeError:
                    response = conn.recv()
                if isinstance(response, str):
                    raise RuntimeError(f"Error from {name} server:\n{response}")
                return response

            def _connect_once(self) -> Any:
                headers = {"Authorization": f"Api-Key {self._api_key}"} if self._api_key else None
                connect_kwargs = {
                    "compression": None,
                    "max_size": None,
                    "open_timeout": min(request_timeout_s, connect_timeout_s),
                }
                try:
                    return websocket_client_policy.websockets.sync.client.connect(
                        self._uri,
                        additional_headers=headers,
                        **connect_kwargs,
                    )
                except TypeError:
                    return websocket_client_policy.websockets.sync.client.connect(
                        self._uri,
                        extra_headers=headers,
                        **connect_kwargs,
                    )

            def _wait_for_server(self) -> tuple[Any, dict[str, Any]]:
                logging.info("Waiting for %s at %s...", name, self._uri)
                start = time.monotonic()
                last_exc: Exception | None = None
                while time.monotonic() - start <= connect_timeout_s:
                    try:
                        conn = self._connect_once()
                        metadata = websocket_client_policy.msgpack_numpy.unpackb(
                            self._recv_with_timeout(conn)
                        )
                        return conn, metadata
                    except Exception as exc:
                        last_exc = exc
                        logging.debug("Still waiting for %s at %s: %s", name, self._uri, exc)
                        time.sleep(0.5)
                raise RuntimeError(
                    f"Timed out connecting {name} to {self._uri} after {connect_timeout_s:.1f}s."
                ) from last_exc

            def infer(self, obs: dict[str, Any]) -> dict[str, Any]:
                data = self._packer.pack(obs)
                self._ws.send(data)
                return websocket_client_policy.msgpack_numpy.unpackb(
                    self._recv_with_timeout(self._ws)
                )

        self._policy = _FiniteWebsocketClientPolicy(host=self.host, port=None, api_key=self.api_key)
        self.metadata = dict(self._policy.get_server_metadata())
        logging.info("%s connected to %s metadata=%s", self.name, self.host, self.metadata)

    def get_server_metadata(self) -> dict[str, Any]:
        return self.metadata

    def reset(self) -> None:
        if self._policy is not None:
            self._policy.reset()

    def close(self) -> None:
        if self._policy is None:
            return
        ws = getattr(self._policy, "_ws", None)
        if ws is not None:
            ws.close()
        self._policy = None

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        if self._policy is None:
            raise RuntimeError(f"{self.name} is not connected.")
        return self._policy.infer(observation)


class RemoteOpenPiPolicy:
    """Action-chunk and action-name adapter around an openpi-client policy."""

    def __init__(
        self,
        *,
        host: str,
        port: int,
        action_names: Sequence[str],
        action_horizon: int,
        connect_timeout_s: float,
        request_timeout_s: float,
        action_offset: int = 0,
        prefetch_remaining_steps: int = 0,
        api_key: str | None = None,
        name: str = "policy",
    ) -> None:
        self.action_names = tuple(action_names)
        self.action_horizon = action_horizon
        self.action_offset = action_offset
        self.prefetch_remaining_steps = max(0, prefetch_remaining_steps)
        self.name = name
        self.client = FiniteOpenPiClientPolicy(
            host=host,
            port=port,
            connect_timeout_s=connect_timeout_s,
            request_timeout_s=request_timeout_s,
            api_key=api_key,
            name=name,
        )
        self.prefetch_client = (
            FiniteOpenPiClientPolicy(
                host=host,
                port=port,
                connect_timeout_s=connect_timeout_s,
                request_timeout_s=request_timeout_s,
                api_key=api_key,
                name=f"{name}_prefetch",
            )
            if self.prefetch_remaining_steps > 0
            else None
        )
        self._prefetch_executor = (
            ThreadPoolExecutor(max_workers=1, thread_name_prefix=f"{name}_prefetch")
            if self.prefetch_remaining_steps > 0
            else None
        )
        self._prefetch_future: Future | None = None
        self._chunk: np.ndarray | None = None
        self._chunk_index = 0
        self._chunk_id = 0
        self._last_received_steps = 0

    @property
    def metadata(self) -> dict[str, Any]:
        return self.client.get_server_metadata()

    def connect(self) -> None:
        self.client.connect()
        if self.prefetch_client is not None:
            self.prefetch_client.connect()
        expected_dim = self.action_offset + len(self.action_names)
        logging.info(
            "%s expected action slice=[%s:%s] from returned `actions`.",
            self.name,
            self.action_offset,
            expected_dim,
        )

    def reset(self) -> None:
        self.client.reset()
        if self.prefetch_client is not None:
            self.prefetch_client.reset()
        if self._prefetch_future is not None:
            self._prefetch_future.cancel()
            self._prefetch_future = None
        self._chunk = None
        self._chunk_index = 0
        self._chunk_id = 0

    def close(self) -> None:
        if self._prefetch_future is not None:
            self._prefetch_future.cancel()
            self._prefetch_future = None
        if self._prefetch_executor is not None:
            self._prefetch_executor.shutdown(wait=False, cancel_futures=True)
        if self.prefetch_client is not None:
            self.prefetch_client.close()
        self.client.close()

    def _fetch_actions(
        self,
        observation: dict[str, Any],
        client: FiniteOpenPiClientPolicy,
    ) -> np.ndarray:
        result = client.infer(observation)
        if "actions" not in result:
            raise KeyError(f"{self.name} response has no 'actions' key. Got keys: {sorted(result)}")
        actions = np.asarray(result["actions"], dtype=np.float32)
        if actions.ndim == 1:
            actions = actions[None, :]
        if actions.ndim != 2:
            raise ValueError(f"{self.name} actions must have shape [horizon, dim], got {actions.shape}.")
        self._last_received_steps = actions.shape[0]
        if actions.shape[0] > self.action_horizon:
            actions = actions[: self.action_horizon]
        elif actions.shape[0] < self.action_horizon:
            logging.warning(
                "%s returned only %s action step(s), less than requested execute horizon %s.",
                self.name,
                actions.shape[0],
                self.action_horizon,
            )
        return actions

    def _install_next_chunk(self, observation: dict[str, Any]) -> None:
        actions: np.ndarray | None = None
        if self._prefetch_future is not None:
            try:
                if not self._prefetch_future.done():
                    logging.debug("%s waiting for prefetched action chunk.", self.name)
                actions = self._prefetch_future.result()
                logging.debug("%s using prefetched action chunk shape=%s.", self.name, actions.shape)
            except Exception:
                logging.exception("%s prefetched action chunk failed; fetching synchronously.", self.name)
            finally:
                self._prefetch_future = None

        if actions is None:
            actions = self._fetch_actions(observation, self.client)

        self._chunk = actions
        self._chunk_index = 0
        self._chunk_id += 1
        logging.info(
            "%s action chunk #%s received_steps=%s action_dim=%s execute_steps=%s.",
            self.name,
            self._chunk_id,
            self._last_received_steps,
            actions.shape[1],
            len(self._chunk),
        )

    def _maybe_prefetch(self, observation: dict[str, Any]) -> None:
        if (
            self.prefetch_client is None
            or self._prefetch_executor is None
            or self._chunk is None
            or self._prefetch_future is not None
        ):
            return
        remaining = len(self._chunk) - self._chunk_index
        if remaining <= 0 or remaining > self.prefetch_remaining_steps:
            return
        logging.debug("%s prefetching next chunk with %s step(s) remaining.", self.name, remaining)
        self._prefetch_future = self._prefetch_executor.submit(
            self._fetch_actions,
            observation,
            self.prefetch_client,
        )

    def infer_next(self, observation: dict[str, Any]) -> PolicyStep:
        if self._chunk is None or self._chunk_index >= len(self._chunk):
            if self._chunk is not None:
                logging.info(
                    "%s action chunk #%s completed executed_steps=%s/%s.",
                    self.name,
                    self._chunk_id,
                    self._chunk_index,
                    len(self._chunk),
                )
            self._install_next_chunk(observation)

        vector = np.asarray(self._chunk[self._chunk_index], dtype=np.float32)
        chunk_step = self._chunk_index
        self._chunk_index += 1
        logging.debug(
            "%s executing action chunk #%s step=%s/%s.",
            self.name,
            self._chunk_id,
            chunk_step + 1,
            len(self._chunk),
        )
        self._maybe_prefetch(observation)
        expected_dim = self.action_offset + len(self.action_names)
        if vector.shape[0] < expected_dim:
            raise ValueError(
                f"{self.name} action dim {vector.shape[0]} is smaller than expected "
                f"{expected_dim} for offset={self.action_offset} and {len(self.action_names)} values."
            )
        sliced = vector[self.action_offset : expected_dim]
        action = {
            name: float(value)
            for name, value in zip(self.action_names, sliced, strict=True)
        }
        return PolicyStep(
            action=action,
            raw_vector=vector,
            source=self.name,
            chunk_id=self._chunk_id,
            chunk_step=chunk_step,
            chunk_len=len(self._chunk),
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Cobot Magic HIL runner for an openpi joint websocket policy server."
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--joint-port", type=int, default=DEFAULT_JOINT_PORT)
    parser.add_argument("--ee-port", type=int, default=DEFAULT_EE_PORT)
    parser.add_argument("--api-key", default=None)
    parser.add_argument("--connect-timeout-s", type=float, default=10.0)
    parser.add_argument("--request-timeout-s", type=float, default=30.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--num-episodes", type=int, default=1)
    parser.add_argument("--episode-time-s", type=float, default=60.0)
    parser.add_argument("--reset-time-s", type=float, default=3.0)
    parser.add_argument("--task", default="put cube in drawer")
    parser.add_argument("--input-format", choices=("aloha", "lerobot"), default="aloha")
    parser.add_argument(
        "--openpi-state-format",
        choices=("observation56", "action28", "joint14"),
        default="observation56",
        help=(
            "State vector sent under the `state` key when --input-format=aloha. "
            "The current remote Cobot Magic OpenPI server expects observation56 and "
            "selects its own joint-position indices server-side."
        ),
    )
    parser.add_argument("--image-size", type=int, default=224)
    parser.add_argument("--action-horizon", type=int, default=8)
    parser.add_argument(
        "--prefetch-remaining-steps",
        type=int,
        default=0,
        help="Start fetching the next OpenPI action chunk when this many steps remain. Use 0 to disable.",
    )
    parser.add_argument("--warmup-inferences", type=int, default=1)
    parser.add_argument("--send-actions", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--record", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--policy-relative-limit",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Apply robot.max_relative_target clipping to OpenPI policy actions. Disabled by default.",
    )
    parser.add_argument("--repo-id", default="local/openpi_cobot_magic_hil")
    parser.add_argument("--dataset-root", default=str(REPO_ROOT / "data" / "openpi_cobot_magic_hil" / "lerobot"))
    parser.add_argument("--resume", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--force-overwrite", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--use-videos", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--vcodec", default="h264", choices=("h264", "hevc", "libsvtav1"))
    parser.add_argument("--image-writer-processes", type=int, default=0)
    parser.add_argument("--image-writer-threads", type=int, default=6)
    parser.add_argument("--enable-ee-shadow", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--ee-action-offset", type=int, default=0)
    parser.add_argument(
        "--swap-joint-arms",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Interpret remote OpenPI joint actions as right-arm then left-arm. "
            "The local Cobot Magic action schema is left-arm then right-arm."
        ),
    )
    parser.add_argument("--enable-teleop", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--mirror-policy-to-leader", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--intervention-key", default="i")
    parser.add_argument("--success-key", default="s")
    parser.add_argument("--failure-key", default="f")
    parser.add_argument("--default-episode-success", choices=("success", "failure"), default=None)
    parser.add_argument("--require-episode-label", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--reset-pose-path", default=None)
    parser.add_argument("--reset-before-episode", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--reset-after-episode", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--reset-on-exit", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--reset-duration-s", type=float, default=5.0)
    parser.add_argument("--dry-run-steps", type=int, default=0)
    parser.add_argument("--log-policy-steps", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def make_features(*, use_videos: bool, include_collector_policy_id: bool = True) -> dict[str, dict[str, Any]]:
    image_dtype = "video" if use_videos else "image"
    features: dict[str, dict[str, Any]] = {
        ACTION: {"dtype": "float32", "shape": (len(ACTION_NAMES),), "names": ACTION_NAMES},
        "observation.state": {"dtype": "float32", "shape": (len(STATE_NAMES),), "names": STATE_NAMES},
        "complementary_info.policy_action": {
            "dtype": "float32",
            "shape": (len(ACTION_NAMES),),
            "names": ACTION_NAMES,
        },
        "complementary_info.ee_shadow_action": {
            "dtype": "float32",
            "shape": (len(EE_ACTION_NAMES),),
            "names": EE_ACTION_NAMES,
        },
        "complementary_info.is_intervention": {
            "dtype": "float32",
            "shape": (1,),
            "names": ["is_intervention"],
        },
        "complementary_info.state": {"dtype": "float32", "shape": (1,), "names": ["state"]},
    }
    if include_collector_policy_id:
        features["complementary_info.collector_policy_id"] = {
            "dtype": "string",
            "shape": (1,),
            "names": list(COLLECTOR_POLICY_ID_NAMES),
        }
    for camera_name in CAMERA_NAMES:
        features[f"observation.images.{camera_name}"] = {
            "dtype": image_dtype,
            "shape": (480, 640, 3),
            "names": ["height", "width", "channels"],
        }
    return features


def make_dataset(args: argparse.Namespace) -> Any | None:
    if not args.record or args.dry_run_steps > 0:
        return None

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(args.dataset_root).expanduser()
    features = make_features(use_videos=args.use_videos)
    if args.force_overwrite and root.exists() and not args.resume:
        logging.warning("Removing existing dataset root because --force-overwrite was set: %s", root)
        shutil.rmtree(root)

    if args.resume:
        dataset = LeRobotDataset(
            args.repo_id,
            root=root,
            batch_encoding_size=1,
            vcodec=args.vcodec,
        )
        if len(dataset.meta.video_keys) > 0:
            dataset.start_image_writer(
                num_processes=args.image_writer_processes,
                num_threads=args.image_writer_threads,
            )
        return dataset

    if root.exists():
        raise FileExistsError(
            f"Dataset root already exists: {root}. Use --resume to append or --force-overwrite to recreate."
        )
    return LeRobotDataset.create(
        args.repo_id,
        fps=args.fps,
        root=root,
        robot_type="cobot_magic_ros_follower",
        features=features,
        use_videos=args.use_videos,
        image_writer_processes=args.image_writer_processes,
        image_writer_threads=args.image_writer_threads,
        batch_encoding_size=1,
        vcodec=args.vcodec,
    )


def make_robot(args: argparse.Namespace) -> Any:
    from lerobot.robots.cobot_magic_ros.cobot_magic_ros import CobotMagicRosFollower
    from lerobot.robots.cobot_magic_ros.config_cobot_magic_ros import CobotMagicRosFollowerConfig

    cfg = CobotMagicRosFollowerConfig(
        id="openpi_cobot_magic_hil_follower",
        sync_gripper=True,
        send_actions=args.send_actions,
        read_timeout_s=5.0,
    )
    return CobotMagicRosFollower(cfg)


def make_teleop() -> Any:
    from lerobot.teleoperators.cobot_magic_ros.cobot_magic_ros import CobotMagicRosLeader
    from lerobot.teleoperators.cobot_magic_ros.config_cobot_magic_ros import CobotMagicRosLeaderConfig

    cfg = CobotMagicRosLeaderConfig(
        id="openpi_cobot_magic_hil_leader",
        sync_gripper=True,
        manual_control=False,
        relative_takeover=True,
        read_timeout_s=1.0,
    )
    return CobotMagicRosLeader(cfg)


def observation_state_vector(obs: dict[str, Any]) -> np.ndarray:
    return np.asarray([float(obs.get(name, 0.0)) for name in STATE_NAMES], dtype=np.float32)


def action_state_vector(obs: dict[str, Any]) -> np.ndarray:
    return np.asarray([float(obs.get(name, 0.0)) for name in ACTION_NAMES], dtype=np.float32)


def aloha_state_vector(obs: dict[str, Any]) -> np.ndarray:
    return np.asarray([float(obs.get(name, 0.0)) for name in JOINT_ACTION_NAMES], dtype=np.float32)


def openpi_state_vector(obs: dict[str, Any], args: argparse.Namespace) -> np.ndarray:
    if args.openpi_state_format == "observation56":
        return observation_state_vector(obs)
    if args.openpi_state_format == "action28":
        return action_state_vector(obs)
    if args.openpi_state_format == "joint14":
        return aloha_state_vector(obs)
    raise ValueError(f"Unsupported --openpi-state-format={args.openpi_state_format!r}.")


def current_hold_action(obs: dict[str, Any]) -> dict[str, float]:
    return {name: float(obs.get(name, 0.0)) for name in JOINT_ACTION_NAMES}


def full_action_from_joint_and_observation(
    joint_action: dict[str, float],
    obs: dict[str, Any],
) -> dict[str, float]:
    action = {name: float(joint_action[name]) for name in JOINT_ACTION_NAMES}
    for name in EE_ACTION_NAMES:
        action[name] = float(obs.get(name, 0.0))
    return action


def full_action_from_teleop(teleop_action: dict[str, Any], obs: dict[str, Any]) -> dict[str, float]:
    action: dict[str, float] = {}
    for name in ACTION_NAMES:
        action[name] = float(teleop_action.get(name, obs.get(name, 0.0)))
    return action


def zero_full_action() -> dict[str, float]:
    return {name: 0.0 for name in ACTION_NAMES}


def ee_shadow_action_from_step(step: PolicyStep | None, obs: dict[str, Any]) -> dict[str, float]:
    if step is None:
        return {name: float(obs.get(name, 0.0)) for name in EE_ACTION_NAMES}
    return {name: float(step.action.get(name, obs.get(name, 0.0))) for name in EE_ACTION_NAMES}


def openpi_joint_action_names(args: argparse.Namespace) -> tuple[str, ...]:
    if args.swap_joint_arms:
        return tuple(OPENPI_SWAPPED_JOINT_ACTION_NAMES)
    return tuple(JOINT_ACTION_NAMES)


def _ensure_hwc_uint8(image: np.ndarray) -> np.ndarray:
    image = np.asarray(image)
    if image.ndim != 3:
        raise ValueError(f"Expected image shape [H,W,C] or [C,H,W], got {image.shape}.")
    if image.shape[0] in (1, 3, 4) and image.shape[-1] not in (1, 3, 4):
        image = np.moveaxis(image, 0, -1)
    if image.dtype != np.uint8:
        if np.issubdtype(image.dtype, np.floating):
            image = np.clip(image * 255.0, 0, 255).astype(np.uint8)
        else:
            image = np.clip(image, 0, 255).astype(np.uint8)
    if image.shape[-1] == 4:
        image = image[..., :3]
    if image.shape[-1] == 1:
        image = np.repeat(image, 3, axis=-1)
    return np.ascontiguousarray(image)


def resize_for_openpi(image: np.ndarray, size: int) -> np.ndarray:
    from openpi_client import image_tools

    image = _ensure_hwc_uint8(image)
    return image_tools.convert_to_uint8(image_tools.resize_with_pad(image, size, size))


def build_openpi_observation(obs: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    images_hwc = {name: resize_for_openpi(obs[name], args.image_size) for name in CAMERA_NAMES}
    if args.input_format == "aloha":
        images_chw = {name: np.moveaxis(image, -1, 0) for name, image in images_hwc.items()}
        return {
            "state": openpi_state_vector(obs, args),
            "images": images_chw,
            "prompt": args.task,
        }
    return {
        "observation.state": observation_state_vector(obs),
        "observation.images.cam_high": images_hwc["cam_high"],
        "observation.images.cam_left_wrist": images_hwc["cam_left_wrist"],
        "observation.images.cam_right_wrist": images_hwc["cam_right_wrist"],
        "prompt": args.task,
    }


def add_frame(
    *,
    dataset: Any | None,
    obs: dict[str, Any],
    sent_action: dict[str, Any],
    policy_action: dict[str, float],
    ee_shadow_action: dict[str, float],
    is_intervention: bool,
    intervention_state: float,
    selected_from_policy: bool,
    task: str,
) -> None:
    if dataset is None:
        return
    observation_frame = build_dataset_frame_local(dataset.features, obs, prefix=OBS_STR)
    action_frame = build_dataset_frame_local(dataset.features, sent_action, prefix=ACTION)
    policy_action_frame = build_dataset_frame_local(
        dataset.features,
        policy_action,
        prefix="complementary_info.policy_action",
    )
    ee_shadow_frame = build_dataset_frame_local(
        dataset.features,
        ee_shadow_action,
        prefix="complementary_info.ee_shadow_action",
    )
    frame = {
        **observation_frame,
        **action_frame,
        **policy_action_frame,
        **ee_shadow_frame,
        "task": task,
    }
    if "complementary_info.is_intervention" in dataset.features:
        frame["complementary_info.is_intervention"] = np.array([float(is_intervention)], dtype=np.float32)
    if "complementary_info.state" in dataset.features:
        frame["complementary_info.state"] = np.array([float(intervention_state)], dtype=np.float32)
    if "complementary_info.collector_policy_id" in dataset.features:
        frame["complementary_info.collector_policy_id"] = resolve_collector_policy_id(
            intervention_enabled=True,
            is_intervention=is_intervention,
            selected_from_policy=selected_from_policy,
            policy_id="openpi_joint",
            human_id="human",
        )
    dataset.add_frame(frame)


def run_reset_pause(
    *,
    robot: Any,
    teleop: Any | None,
    duration_s: float,
    fps: int,
) -> None:
    if duration_s <= 0:
        return
    logging.info("Reset pause for %.1fs. Drag/setup the scene now.", duration_s)
    if teleop is not None:
        teleop.set_manual_control(True)
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        obs = robot.get_joint_observation()
        robot.send_action_without_relative_limit(current_hold_action(obs))
        precise_sleep(1.0 / fps)
    if teleop is not None:
        teleop.set_manual_control(False)


def maybe_reset_to_pose(
    *,
    args: argparse.Namespace,
    robot: Any,
    teleop: Any | None,
    reset_pose: dict[str, float] | None,
    leader_reset_pose: dict[str, float] | None = None,
) -> None:
    if reset_pose is None:
        return
    from lerobot.scripts.robot_reset import slow_reset_all_arms_to_pose

    reset_teleop = teleop
    if reset_teleop is not None and leader_reset_pose is None:
        logging.warning("No leader reset pose is available; leaving leader arms untouched during reset.")
        reset_teleop = None

    slow_reset_all_arms_to_pose(
        robot=robot,
        teleop=reset_teleop,
        target_pose=reset_pose,
        teleop_target_pose=leader_reset_pose,
        duration_s=args.reset_duration_s,
        fps=min(args.fps, 30),
    )


def toggle_intervention(
    *,
    teleop: Any | None,
    active: bool,
    joint_policy: RemoteOpenPiPolicy,
    ee_policy: RemoteOpenPiPolicy | None,
) -> bool:
    next_active = not active
    if teleop is not None:
        teleop.set_manual_control(next_active)
    if not next_active:
        joint_policy.reset()
        if ee_policy is not None:
            ee_policy.reset()
    logging.info("Intervention %s.", "ACTIVE" if next_active else "released to policy")
    return next_active


def run_episode(
    *,
    args: argparse.Namespace,
    robot: Any,
    teleop: Any | None,
    joint_policy: RemoteOpenPiPolicy,
    ee_policy: RemoteOpenPiPolicy | None,
    dataset: Any | None,
    events: dict[str, Any],
) -> str | None:
    joint_policy.reset()
    if ee_policy is not None:
        ee_policy.reset()
    if teleop is not None:
        teleop.set_manual_control(False)

    intervention_active = False
    end_time = time.monotonic() + args.episode_time_s
    frame_count = 0
    last_policy_action = zero_full_action()
    last_policy_step_action: dict[str, float] | None = None
    last_ee_shadow_step: PolicyStep | None = None
    last_loop_log = time.monotonic()

    while time.monotonic() < end_time and not events["exit_early"] and not events["stop_recording"]:
        loop_start = time.monotonic()
        if events.get("toggle_intervention"):
            events["toggle_intervention"] = False
            intervention_active = toggle_intervention(
                teleop=teleop,
                active=intervention_active,
                joint_policy=joint_policy,
                ee_policy=ee_policy,
            )

        obs = robot.get_observation()
        policy_obs = build_openpi_observation(obs, args)

        policy_step: PolicyStep | None = None
        ee_step: PolicyStep | None = None
        selected_from_policy = False
        is_intervention = bool(intervention_active)
        state_code = INTERVENTION_STATE_ACTIVE if is_intervention else INTERVENTION_STATE_POLICY

        if is_intervention and teleop is not None:
            teleop_action = teleop.get_action()
            action_to_send = full_action_from_teleop(teleop_action, obs)
            sent_action = robot.send_action_without_relative_limit(action_to_send)
            policy_action_for_storage = last_policy_action
        else:
            try:
                policy_step = joint_policy.infer_next(policy_obs)
                action_to_send = full_action_from_joint_and_observation(policy_step.action, obs)
                policy_action_for_storage = dict(action_to_send)
                last_policy_action = dict(action_to_send)
                if args.policy_relative_limit:
                    sent_action = robot.send_action(action_to_send)
                else:
                    sent_action = robot.send_action_without_relative_limit(action_to_send)
                selected_from_policy = True
                if args.log_policy_steps:
                    if last_policy_step_action is None:
                        max_delta = 0.0
                    else:
                        max_delta = max(
                            abs(action_to_send[name] - last_policy_step_action.get(name, action_to_send[name]))
                            for name in JOINT_ACTION_NAMES
                        )
                    logging.info(
                        "policy_step chunk=%s step=%s/%s max_joint_delta=%.4f loop_ms=%.1f",
                        policy_step.chunk_id,
                        policy_step.chunk_step + 1,
                        policy_step.chunk_len,
                        max_delta,
                        (time.monotonic() - loop_start) * 1000.0,
                    )
                    last_policy_step_action = dict(action_to_send)
                if teleop is not None and args.mirror_policy_to_leader:
                    try:
                        teleop.send_feedback(sent_action)
                    except Exception:
                        logging.exception("Failed to mirror policy action to leader.")
            except Exception:
                logging.exception("Joint policy inference/control failed; holding current joint pose.")
                action_to_send = full_action_from_joint_and_observation(current_hold_action(obs), obs)
                policy_action_for_storage = last_policy_action
                sent_action = robot.send_action_without_relative_limit(action_to_send)

        if ee_policy is not None:
            try:
                ee_step = ee_policy.infer_next(policy_obs)
                last_ee_shadow_step = ee_step
            except Exception:
                logging.exception("EE shadow inference failed; reusing previous EE shadow action.")
                ee_step = last_ee_shadow_step
        ee_shadow_action = ee_shadow_action_from_step(ee_step, obs)

        sent_full_action = {
            name: float(sent_action.get(name, action_to_send.get(name, obs.get(name, 0.0))))
            for name in ACTION_NAMES
        }
        add_frame(
            dataset=dataset,
            obs=obs,
            sent_action=sent_full_action,
            policy_action=policy_action_for_storage,
            ee_shadow_action=ee_shadow_action,
            is_intervention=is_intervention,
            intervention_state=state_code,
            selected_from_policy=selected_from_policy,
            task=args.task,
        )

        frame_count += 1
        now = time.monotonic()
        if now - last_loop_log >= 5.0:
            logging.info(
                "episode frames=%s mode=%s send_actions=%s",
                frame_count,
                "human" if is_intervention else "policy",
                args.send_actions,
            )
            last_loop_log = now

        precise_sleep(max(0.0, (1.0 / args.fps) - (time.monotonic() - loop_start)))

    if events.get("episode_outcome") is not None:
        return str(events["episode_outcome"])
    return None


def run(args: argparse.Namespace) -> None:
    if args.fps <= 0:
        raise ValueError("--fps must be > 0.")
    if args.episode_time_s <= 0:
        raise ValueError("--episode-time-s must be > 0.")
    if args.action_horizon <= 0:
        raise ValueError("--action-horizon must be > 0.")
    if args.require_episode_label and not args.record:
        raise ValueError("--require-episode-label only makes sense with --record.")

    dataset = make_dataset(args)
    robot = make_robot(args)
    teleop = make_teleop() if args.enable_teleop else None
    if args.reset_pose_path:
        from lerobot.scripts.robot_reset import load_optional_named_joint_pose, load_reset_pose

        reset_pose_path = Path(args.reset_pose_path).expanduser()
        reset_pose = load_reset_pose(reset_pose_path)
        leader_reset_pose = load_optional_named_joint_pose(reset_pose_path, "leader_joint_pos")
    else:
        reset_pose = None
        leader_reset_pose = None
    joint_policy = RemoteOpenPiPolicy(
        host=args.host,
        port=args.joint_port,
        action_names=openpi_joint_action_names(args),
        action_horizon=args.action_horizon,
        connect_timeout_s=args.connect_timeout_s,
        request_timeout_s=args.request_timeout_s,
        action_offset=0,
        prefetch_remaining_steps=args.prefetch_remaining_steps,
        api_key=args.api_key,
        name="openpi_joint",
    )
    ee_policy = (
        RemoteOpenPiPolicy(
            host=args.host,
            port=args.ee_port,
            action_names=EE_ACTION_NAMES,
            action_horizon=args.action_horizon,
            connect_timeout_s=args.connect_timeout_s,
            request_timeout_s=args.request_timeout_s,
            action_offset=args.ee_action_offset,
            prefetch_remaining_steps=0,
            api_key=args.api_key,
            name="openpi_ee_shadow",
        )
        if args.enable_ee_shadow
        else None
    )

    listener = None
    try:
        logging.info("Connecting robot and teleop.")
        robot.connect()
        if teleop is not None:
            teleop.connect()
            teleop.set_manual_control(False)

        logging.info("Connecting openpi joint server %s:%s.", args.host, args.joint_port)
        joint_policy.connect()
        if ee_policy is not None:
            logging.info("Connecting openpi EE shadow server %s:%s.", args.host, args.ee_port)
            ee_policy.connect()

        listener, events = init_events_and_keyboard(
            intervention_key=args.intervention_key,
            success_key=args.success_key,
            failure_key=args.failure_key,
        )

        if args.warmup_inferences > 0:
            logging.info("Running %s policy warmup inference(s).", args.warmup_inferences)
            for _ in range(args.warmup_inferences):
                obs = robot.get_observation()
                policy_obs = build_openpi_observation(obs, args)
                joint_policy.infer_next(policy_obs)
                if ee_policy is not None:
                    ee_policy.infer_next(policy_obs)
            joint_policy.reset()
            if ee_policy is not None:
                ee_policy.reset()

        if args.dry_run_steps > 0:
            logging.info("Running dry-run for %s steps.", args.dry_run_steps)
            for _ in range(args.dry_run_steps):
                obs = robot.get_observation()
                policy_obs = build_openpi_observation(obs, args)
                joint_step = joint_policy.infer_next(policy_obs)
                ee_step = ee_policy.infer_next(policy_obs) if ee_policy is not None else None
                logging.info(
                    "dry-run joint_dim=%s ee_dim=%s first_joint=%.4f",
                    joint_step.raw_vector.shape,
                    None if ee_step is None else ee_step.raw_vector.shape,
                    joint_step.raw_vector[0],
                )
                robot.send_action_without_relative_limit(current_hold_action(obs))
                precise_sleep(1.0 / args.fps)
            return

        recorded = 0
        episode_idx = 0
        while recorded < args.num_episodes and not events["stop_recording"]:
            events["exit_early"] = False
            events["rerecord_episode"] = False
            events["toggle_intervention"] = False
            events["episode_outcome"] = None

            if args.reset_before_episode:
                maybe_reset_to_pose(
                    args=args,
                    robot=robot,
                    teleop=teleop,
                    reset_pose=reset_pose,
                    leader_reset_pose=leader_reset_pose,
                )

            logging.info(
                "Starting episode %s/%s. Press '%s' to toggle HIL, '%s' success, '%s' failure.",
                recorded + 1,
                args.num_episodes,
                args.intervention_key,
                args.success_key,
                args.failure_key,
            )
            explicit_label = run_episode(
                args=args,
                robot=robot,
                teleop=teleop,
                joint_policy=joint_policy,
                ee_policy=ee_policy,
                dataset=dataset,
                events=events,
            )

            rerecord = bool(events["rerecord_episode"])
            stop_without_label = bool(events["stop_recording"]) and explicit_label is None
            discard = rerecord or stop_without_label
            episode_success = None
            if args.record and not discard:
                try:
                    episode_success = resolve_episode_success_label(
                        explicit_label=explicit_label,
                        default_label=args.default_episode_success,
                        require_label=args.require_episode_label,
                    )
                except ValueError:
                    logging.warning(
                        "Episode has no success/failure label and --require-episode-label is enabled; discarding."
                    )
                    discard = True

            if args.reset_after_episode and not events["stop_recording"]:
                maybe_reset_to_pose(
                    args=args,
                    robot=robot,
                    teleop=teleop,
                    reset_pose=reset_pose,
                    leader_reset_pose=leader_reset_pose,
                )

            if dataset is not None:
                if discard:
                    dataset.clear_episode_buffer()
                    logging.info("Discarded episode buffer.")
                else:
                    extra = {"episode_success": episode_success} if episode_success is not None else None
                    dataset.save_episode(extra_episode_metadata=extra)
                    recorded += 1
                    logging.info("Saved episode %s.", recorded)
            else:
                recorded += 1

            episode_idx += 1
            if events["stop_recording"]:
                break
            if recorded < args.num_episodes:
                run_reset_pause(robot=robot, teleop=teleop, duration_s=args.reset_time_s, fps=args.fps)

        logging.info("Finished. recorded=%s requested=%s episodes_started=%s", recorded, args.num_episodes, episode_idx)
    finally:
        joint_policy.close()
        if ee_policy is not None:
            ee_policy.close()
        if listener is not None:
            listener.stop()
        if dataset is not None and hasattr(dataset, "stop_image_writer"):
            dataset.stop_image_writer()
        if args.reset_on_exit and reset_pose is not None and robot.is_connected:
            try:
                maybe_reset_to_pose(
                    args=args,
                    robot=robot,
                    teleop=teleop,
                    reset_pose=reset_pose,
                    leader_reset_pose=leader_reset_pose,
                )
            except Exception:
                logging.exception("Failed to reset robot during shutdown.")
        if teleop is not None and teleop.is_connected:
            try:
                teleop.set_manual_control(False)
            except Exception:
                logging.exception("Failed to disable teleop manual control during shutdown.")
            teleop.disconnect()
        if robot.is_connected:
            robot.disconnect()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(message)s",
        force=True,
    )
    if not args.send_actions:
        logging.warning("--send-actions=false: follower command topics will not move the robot.")
    run(args)


if __name__ == "__main__":
    main()
