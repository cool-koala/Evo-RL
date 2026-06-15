#!/usr/bin/env python

from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESET_POSE_PATH = (
    REPO_ROOT
    / "src"
    / "lerobot"
    / "robots"
    / "cobot_magic_ros"
    / "reset_poses"
    / "cobot_magic_ros_x5_initial_pose.json"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Return Cobot Magic/X5 arms to the stored initial pose.")
    parser.add_argument("--reset-pose-path", default=str(DEFAULT_RESET_POSE_PATH))
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--read-timeout-s", type=float, default=5.0)
    parser.add_argument("--no-leader", action="store_true", help="Only reset follower/puppet arms.")
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def make_robot(args: argparse.Namespace) -> Any:
    from lerobot.robots.cobot_magic_ros.cobot_magic_ros import CobotMagicRosFollower
    from lerobot.robots.cobot_magic_ros.config_cobot_magic_ros import CobotMagicRosFollowerConfig

    return CobotMagicRosFollower(
        CobotMagicRosFollowerConfig(
            id="cobot_magic_reset_pose_follower",
            cameras={},
            sync_gripper=True,
            send_actions=True,
            read_timeout_s=args.read_timeout_s,
        )
    )


def make_teleop(args: argparse.Namespace) -> Any:
    from lerobot.teleoperators.cobot_magic_ros.cobot_magic_ros import CobotMagicRosLeader
    from lerobot.teleoperators.cobot_magic_ros.config_cobot_magic_ros import CobotMagicRosLeaderConfig

    return CobotMagicRosLeader(
        CobotMagicRosLeaderConfig(
            id="cobot_magic_reset_pose_leader",
            sync_gripper=True,
            manual_control=False,
            relative_takeover=False,
            read_timeout_s=args.read_timeout_s,
        )
    )


def run(args: argparse.Namespace) -> None:
    if args.duration_s <= 0:
        raise ValueError("--duration-s must be > 0.")
    if args.fps <= 0:
        raise ValueError("--fps must be > 0.")

    from lerobot.scripts.robot_reset import (
        load_optional_named_joint_pose,
        load_reset_pose,
        slow_reset_all_arms_to_pose,
    )

    pose_path = Path(args.reset_pose_path).expanduser()
    follower_pose = load_reset_pose(pose_path)
    leader_pose = None if args.no_leader else load_optional_named_joint_pose(pose_path, "leader_joint_pos")

    robot = make_robot(args)
    teleop = None if args.no_leader else make_teleop(args)
    try:
        logging.info("Connecting Cobot Magic ROS follower.")
        robot.connect()
        if teleop is not None:
            if leader_pose is None:
                logging.warning(
                    "No leader_joint_pos found in %s; leaving leader arms untouched.", pose_path
                )
                teleop = None
            else:
                logging.info("Connecting Cobot Magic ROS leader.")
                teleop.connect()
                teleop.set_manual_control(False)

        slow_reset_all_arms_to_pose(
            robot=robot,
            teleop=teleop,
            target_pose=follower_pose,
            teleop_target_pose=leader_pose,
            duration_s=args.duration_s,
            fps=args.fps,
        )
    finally:
        if teleop is not None and teleop.is_connected:
            try:
                teleop.set_manual_control(False)
            except Exception:
                logging.exception("Failed to disable leader manual control during shutdown.")
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
    run(args)


if __name__ == "__main__":
    main()
