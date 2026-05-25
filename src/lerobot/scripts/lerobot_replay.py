# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

"""
Replays the actions of an episode from a dataset on a robot.

Examples:

```shell
lerobot-replay \
    --robot.type=so100_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.id=black \
    --dataset.repo_id=aliberts/record-test \
    --dataset.episode=0
```

Example replay with bimanual so100:
```shell
lerobot-replay \
  --robot.type=bi_so_follower \
  --robot.left_arm_port=/dev/tty.usbmodem5A460851411 \
  --robot.right_arm_port=/dev/tty.usbmodem5A460812391 \
  --robot.id=bimanual_follower \
  --dataset.repo_id=${HF_USER}/bimanual-so100-handover-cube \
  --dataset.episode=0
```

"""

import logging
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from pprint import pformat

from lerobot.configs import parser
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.processor import (
    make_default_robot_action_processor,
)
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    bi_openarm_follower,
    bi_piper_follower,
    bi_so_follower,
    cobot_magic,
    cobot_magic_ros,
    earthrover_mini_plus,
    hope_jr,
    koch_follower,
    make_robot_from_config,
    omx_follower,
    openarm_follower,
    piper_follower,
    reachy2,
    so_follower,
    unitree_g1,
)
from lerobot.scripts.robot_reset import (
    default_reset_pose_path,
    load_reset_pose,
    make_zero_pose,
    slow_reset_all_arms_to_pose,
)
from lerobot.utils.constants import ACTION
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import (
    init_logging,
    log_say,
)

COBOT_MAGIC_ROS_PROJECT_ZERO_POSE_PATH = (
    Path(__file__).resolve().parents[1]
    / "robots"
    / "cobot_magic_ros"
    / "reset_poses"
    / "cobot_magic_ros_x5_initial_pose.json"
)


@dataclass
class DatasetReplayConfig:
    # Dataset identifier. By convention it should match '{hf_username}/{dataset_name}' (e.g. `lerobot/test`).
    repo_id: str
    # Episode to replay.
    episode: int
    # Root directory where the dataset will be stored (e.g. 'dataset/path').
    root: str | Path | None = None
    # Limit the frames per second. By default, uses the policy fps.
    fps: int = 30


@dataclass
class ReplayConfig:
    robot: RobotConfig
    dataset: DatasetReplayConfig
    # Use vocal synthesis to read events.
    play_sounds: bool = True
    # Use a fixed zero/reset pose before and after replay.
    reset_to_zero_pose: bool = False
    # Optional joint reset pose JSON. If omitted, a per robot type/id path under HF cache is used.
    reset_pose_path: str | Path | None = None
    # Move the robot to the stored reset pose before sending replay actions.
    reset_before_replay: bool = False
    # Move the robot to the stored reset pose after the replay finishes.
    reset_after_replay: bool = False
    # Duration used for slow reset-pose interpolation.
    reset_duration_s: float = 5.0
    # If true, fail instead of jumping when the first dataset action is far from the reset pose.
    require_replay_start_pose: bool = False
    # Tolerance for checking that the first replay action matches the reset pose.
    reset_pose_tolerance: float = 0.05

    def __post_init__(self):
        if self.reset_duration_s <= 0:
            raise ValueError("`reset_duration_s` must be > 0.")
        if self.reset_pose_tolerance < 0:
            raise ValueError("`reset_pose_tolerance` must be >= 0.")


def _replay_reset_pose_path(cfg: ReplayConfig) -> Path:
    if cfg.reset_pose_path is not None:
        return Path(cfg.reset_pose_path).expanduser()
    robot_type = cfg.robot.type if hasattr(cfg.robot, "type") else type(cfg.robot).__name__
    return default_reset_pose_path(robot_type, cfg.robot.id)


def _replay_zero_pose_path(cfg: ReplayConfig) -> Path | None:
    if cfg.reset_pose_path is not None:
        return Path(cfg.reset_pose_path).expanduser()
    robot_type = cfg.robot.type if hasattr(cfg.robot, "type") else type(cfg.robot).__name__
    if robot_type == "cobot_magic_ros_follower":
        return COBOT_MAGIC_ROS_PROJECT_ZERO_POSE_PATH
    return None


def _first_replay_action_pose(dataset: LeRobotDataset, actions) -> dict[str, float]:
    if len(actions) == 0:
        raise RuntimeError("Cannot use replay zero/reset pose: selected episode has no actions.")
    action_names = dataset.features[ACTION]["names"]
    first_action_array = actions[0][ACTION]
    reset_pose = {
        name: float(first_action_array[idx]) for idx, name in enumerate(action_names) if name.endswith(".pos")
    }
    if not reset_pose:
        raise RuntimeError("Cannot use replay zero/reset pose: first action has no '.pos' joints.")
    return reset_pose


def _load_replay_reset_pose_if_requested(
    cfg: ReplayConfig,
    robot: Robot,
    dataset: LeRobotDataset,
    actions,
) -> dict[str, float] | None:
    if not (
        cfg.reset_to_zero_pose
        or cfg.reset_before_replay
        or cfg.reset_after_replay
        or cfg.require_replay_start_pose
    ):
        return None
    if cfg.reset_to_zero_pose:
        pose_path = _replay_zero_pose_path(cfg)
        if pose_path is not None:
            if not pose_path.is_file():
                raise FileNotFoundError(f"Fixed zero pose file does not exist: {pose_path}")
            logging.info("Using fixed project zero/reset pose from %s.", pose_path)
            return load_reset_pose(pose_path)
        logging.info("Using literal joint-zero pose as the reset pose.")
        return make_zero_pose(robot)
    pose_path = _replay_reset_pose_path(cfg)
    if not pose_path.is_file():
        raise FileNotFoundError(
            f"Reset pose file does not exist: {pose_path}. "
            "Capture it during recording with `--capture_reset_pose=true`, "
            "or pass `--reset_pose_path` to an existing pose."
        )
    return load_reset_pose(pose_path)


def _assert_replay_starts_from_reset_pose_if_required(
    *,
    cfg: ReplayConfig,
    dataset: LeRobotDataset,
    actions,
    reset_pose: dict[str, float] | None,
) -> None:
    if reset_pose is None or not (cfg.reset_to_zero_pose or cfg.require_replay_start_pose):
        return
    if len(actions) == 0:
        return
    action_names = dataset.features[ACTION]["names"]
    first_action_array = actions[0][ACTION]
    first_action = {name: float(first_action_array[idx]) for idx, name in enumerate(action_names)}
    joint_keys = [key for key in reset_pose if key in first_action]
    if not joint_keys:
        return
    max_delta = max(abs(first_action[key] - float(reset_pose[key])) for key in joint_keys)
    if max_delta > cfg.reset_pose_tolerance:
        raise RuntimeError(
            "Replay must start from the fixed reset/zero pose, but the first dataset action is too far "
            f"({max_delta:.3f} rad > {cfg.reset_pose_tolerance:.3f} rad). "
            "Use a dataset recorded from the saved start pose, or disable the start-pose check after confirming safety."
        )


def _slow_replay_reset_if_requested(
    *,
    cfg: ReplayConfig,
    robot: Robot,
    reset_pose: dict[str, float] | None,
) -> None:
    if reset_pose is None:
        return
    slow_reset_all_arms_to_pose(
        robot=robot,
        teleop=None,
        target_pose=reset_pose,
        duration_s=cfg.reset_duration_s,
        fps=cfg.dataset.fps,
    )


@parser.wrap()
def replay(cfg: ReplayConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    robot_action_processor = make_default_robot_action_processor()

    robot = make_robot_from_config(cfg.robot)
    dataset = LeRobotDataset(cfg.dataset.repo_id, root=cfg.dataset.root, episodes=[cfg.dataset.episode])

    # Filter dataset to only include frames from the specified episode since episodes are chunked in dataset V3.0
    episode_frames = dataset.hf_dataset.filter(lambda x: x["episode_index"] == cfg.dataset.episode)
    actions = episode_frames.select_columns(ACTION)
    reset_pose = _load_replay_reset_pose_if_requested(cfg, robot, dataset, actions)
    _assert_replay_starts_from_reset_pose_if_required(
        cfg=cfg,
        dataset=dataset,
        actions=actions,
        reset_pose=reset_pose,
    )

    robot.connect()

    try:
        if cfg.reset_before_replay:
            _slow_replay_reset_if_requested(cfg=cfg, robot=robot, reset_pose=reset_pose)
        log_say("Replaying episode", cfg.play_sounds, blocking=True)
        for idx in range(len(episode_frames)):
            start_episode_t = time.perf_counter()

            action_array = actions[idx][ACTION]
            action = {}
            for i, name in enumerate(dataset.features[ACTION]["names"]):
                action[name] = action_array[i]

            robot_obs = robot.get_observation()

            processed_action = robot_action_processor((action, robot_obs))

            _ = robot.send_action(processed_action)

            dt_s = time.perf_counter() - start_episode_t
            precise_sleep(max(1 / dataset.fps - dt_s, 0.0))
        if cfg.reset_after_replay:
            _slow_replay_reset_if_requested(cfg=cfg, robot=robot, reset_pose=reset_pose)
    finally:
        robot.disconnect()


def main():
    register_third_party_plugins()
    replay()


if __name__ == "__main__":
    main()
