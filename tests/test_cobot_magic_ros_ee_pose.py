from __future__ import annotations

from types import SimpleNamespace

import pytest

from lerobot.robots.cobot_magic_ros.cobot_magic_ros import CobotMagicRosFollower
from lerobot.robots.cobot_magic_ros.config_cobot_magic_ros import CobotMagicRosFollowerConfig
from lerobot.utils.cobot_magic_ros import (
    ROS_EE_COMMAND_NAMES,
    build_ros_ee_positions,
    clamp_ee_pose_step,
    make_joint_state_message,
)


class _FakeClock:
    def now(self):
        return self

    def to_msg(self):
        return "stamp"


class _FakeNode:
    def get_clock(self):
        return _FakeClock()


class _FakeHeader:
    pass


class _FakeJointState:
    def __init__(self):
        self.header = None
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture(autouse=True)
def fake_ros_node(monkeypatch):
    import lerobot.robots.cobot_magic_ros.cobot_magic_ros as follower_module
    import lerobot.utils.cobot_magic_ros as cobot_magic_ros

    monkeypatch.setattr(cobot_magic_ros, "_ROS2_NODE", _FakeNode())
    monkeypatch.setattr(follower_module, "spin_ros_once", lambda *args, **kwargs: None)


def _fake_ros():
    return SimpleNamespace(JointState=_FakeJointState, Header=_FakeHeader)


def test_build_ros_ee_positions_requires_all_keys():
    action = {
        "left_ee.x": 0.1,
        "left_ee.y": 0.2,
        "left_ee.z": 0.3,
        "left_ee.wx": 0.4,
        "left_ee.wy": 0.5,
        "left_ee.wz": 0.6,
        "left_ee.gripper_pos": 0.7,
    }

    positions, sent = build_ros_ee_positions(action, "left")

    assert positions == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    assert sent == action
    with pytest.raises(KeyError):
        build_ros_ee_positions({"left_ee.x": 0.1}, "left")


def test_make_ee_joint_state_message_uses_ee_names():
    msg = make_joint_state_message(_fake_ros(), [1, 2, 3, 4, 5, 6, 7], names=ROS_EE_COMMAND_NAMES)

    assert msg.name == list(ROS_EE_COMMAND_NAMES)
    assert msg.position == pytest.approx([1, 2, 3, 4, 5, 6, 7])
    assert msg.header.stamp == "stamp"


def test_clamp_ee_pose_step_limits_delta_and_workspace():
    target = [1.0, -1.0, 0.8, 2.0, -2.0, 0.5, 1.0]
    current = [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.2]

    clamped = clamp_ee_pose_step(
        target,
        current,
        max_xyz_step_m=0.1,
        max_rot_step=0.2,
        max_gripper_step=0.05,
        workspace_min=(-0.05, -0.05, 0.0),
        workspace_max=(0.05, 0.05, 0.45),
    )

    assert clamped == pytest.approx([0.05, -0.05, 0.45, 0.2, -0.2, 0.2, 0.25])


def test_follower_ee_mode_publishes_ee_topics_only():
    cfg = CobotMagicRosFollowerConfig(
        id="test",
        control_mode="ee_pose",
        send_actions=True,
        cameras={},
    )
    robot = CobotMagicRosFollower(cfg)
    robot._ros = _fake_ros()
    robot._left_ee_command_publisher = _FakePublisher()
    robot._right_ee_command_publisher = _FakePublisher()
    robot._left_command_publisher = _FakePublisher()
    robot._right_command_publisher = _FakePublisher()
    action = {
        **{f"left_ee.{key}": float(idx) for idx, key in enumerate(("x", "y", "z", "wx", "wy", "wz"))},
        "left_ee.gripper_pos": 6.0,
        **{
            f"right_ee.{key}": float(idx + 10)
            for idx, key in enumerate(("x", "y", "z", "wx", "wy", "wz"))
        },
        "right_ee.gripper_pos": 16.0,
    }

    sent = robot._send_action(action)

    assert robot._left_command_publisher.messages == []
    assert robot._right_command_publisher.messages == []
    assert robot._left_ee_command_publisher.messages[0].name == list(ROS_EE_COMMAND_NAMES)
    assert robot._right_ee_command_publisher.messages[0].position[-1] == pytest.approx(16.0)
    assert sent["left_ee.x"] == pytest.approx(0.0)
    assert sent["right_ee.gripper_pos"] == pytest.approx(16.0)
