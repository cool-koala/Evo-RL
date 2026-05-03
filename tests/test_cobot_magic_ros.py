import json
import sys
from types import SimpleNamespace

import draccus
import numpy as np
import pytest

from lerobot.robots.cobot_magic_ros import CobotMagicRosCameraConfig, CobotMagicRosFollowerConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.scripts.lerobot_record import (
    RecordConfig,
    _assert_teleop_matches_reset_pose_if_required,
    _prepare_record_reset_pose,
)
from lerobot.scripts.lerobot_replay import ReplayConfig, _first_replay_action_pose
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig, _run_startup_sync_if_requested
from lerobot.teleoperators.cobot_magic_ros import CobotMagicRosLeaderConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.utils.constants import ACTION


class FakeHeader:
    def __init__(self):
        self.stamp = None


class FakeBool:
    def __init__(self):
        self.data = False


class FakeJointState:
    def __init__(self):
        self.header = FakeHeader()
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []


class FakeImage:
    def __init__(self, data: bytes = b"", height: int = 0, width: int = 0, encoding: str = "rgb8"):
        self.header = FakeHeader()
        self.data = data
        self.height = height
        self.width = width
        self.encoding = encoding


class FakePublisher:
    def __init__(self, topic: str):
        self.topic = topic
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeSubscriber:
    def __init__(self, topic: str, callback):
        self.topic = topic
        self.callback = callback
        self.unregistered = False

    def emit(self, msg):
        self.callback(msg)

    def unregister(self):
        self.unregistered = True


class FakeRospy:
    def __init__(self):
        self.initialized = False
        self.publishers = {}
        self.subscribers = {}
        self.core = SimpleNamespace(is_initialized=lambda: self.initialized)
        self.Time = SimpleNamespace(now=lambda: 123.456)

    def init_node(self, *args, **kwargs):
        del args, kwargs
        self.initialized = True

    def Publisher(self, topic, msg_type, queue_size=10):  # noqa: N802
        del msg_type, queue_size
        publisher = FakePublisher(topic)
        self.publishers[topic] = publisher
        return publisher

    def Subscriber(self, topic, msg_type, callback, queue_size=10, tcp_nodelay=True):  # noqa: N802
        del msg_type, queue_size, tcp_nodelay
        subscriber = FakeSubscriber(topic, callback)
        self.subscribers.setdefault(topic, []).append(subscriber)
        return subscriber


def install_fake_ros(monkeypatch):
    fake_rospy = FakeRospy()
    monkeypatch.setitem(sys.modules, "rospy", fake_rospy)
    monkeypatch.setitem(sys.modules, "std_msgs", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(Bool=FakeBool, Header=FakeHeader))
    monkeypatch.setitem(sys.modules, "sensor_msgs", SimpleNamespace())
    monkeypatch.setitem(
        sys.modules,
        "sensor_msgs.msg",
        SimpleNamespace(JointState=FakeJointState, Image=FakeImage),
    )
    return fake_rospy


def make_joint_state(position, velocity=None, effort=None):
    msg = FakeJointState()
    msg.position = list(position)
    msg.velocity = list(velocity if velocity is not None else [0.0] * len(position))
    msg.effort = list(effort if effort is not None else [0.0] * len(position))
    return msg


def emit_topic(fake_rospy, topic: str, msg):
    for subscriber in fake_rospy.subscribers[topic]:
        subscriber.emit(msg)


def test_cobot_magic_ros_parse_teleoperate_config():
    args = [
        "--robot.type=cobot_magic_ros_follower",
        "--robot.id=ros_follower",
        "--robot.cameras={}",
        "--teleop.type=cobot_magic_ros_leader",
        "--teleop.id=ros_leader",
        "--teleop.startup_sync=true",
        "--teleop.startup_sync_duration_s=2.5",
    ]

    cfg = draccus.parse(config_class=TeleoperateConfig, config_path=None, args=args)

    assert cfg.robot.type == "cobot_magic_ros_follower"
    assert cfg.teleop.type == "cobot_magic_ros_leader"
    assert cfg.teleop.startup_sync is True
    assert cfg.teleop.startup_sync_duration_s == pytest.approx(2.5)


def test_cobot_magic_ros_parse_record_config():
    args = [
        "--robot.type=cobot_magic_ros_follower",
        "--robot.id=ros_follower",
        "--robot.cameras={}",
        "--teleop.type=cobot_magic_ros_leader",
        "--teleop.id=ros_leader",
        "--dataset.repo_id=dummy/dummy",
        "--dataset.single_task=test",
        "--dataset.num_episodes=1",
        "--dataset.episode_time_s=1",
        "--dataset.reset_time_s=1",
        "--dataset.push_to_hub=false",
        "--reset_to_zero_pose=true",
        "--reset_before_record=true",
        "--reset_after_episode=true",
        "--reset_duration_s=4.5",
        "--reset_pose_tolerance=0.03",
        "--hil_leader_mode=parked",
        "--hil_leader_sync_duration_s=2.0",
        "--hil_leader_return_duration_s=3.0",
    ]

    cfg = draccus.parse(config_class=RecordConfig, config_path=None, args=args)

    assert cfg.robot.type == "cobot_magic_ros_follower"
    assert cfg.teleop.type == "cobot_magic_ros_leader"
    assert cfg.reset_to_zero_pose is True
    assert cfg.reset_before_record is True
    assert cfg.reset_after_episode is True
    assert cfg.reset_duration_s == pytest.approx(4.5)
    assert cfg.reset_pose_tolerance == pytest.approx(0.03)
    assert cfg.hil_leader_mode == "parked"
    assert cfg.hil_leader_sync_duration_s == pytest.approx(2.0)
    assert cfg.hil_leader_return_duration_s == pytest.approx(3.0)


def test_cobot_magic_ros_parse_replay_reset_config():
    args = [
        "--robot.type=cobot_magic_ros_follower",
        "--robot.id=ros_follower",
        "--robot.cameras={}",
        "--dataset.repo_id=dummy/dummy",
        "--dataset.episode=0",
        "--reset_to_zero_pose=true",
        "--reset_before_replay=true",
        "--reset_after_replay=true",
        "--require_replay_start_pose=true",
        "--reset_duration_s=6.0",
        "--reset_pose_tolerance=0.02",
    ]

    cfg = draccus.parse(config_class=ReplayConfig, config_path=None, args=args)

    assert cfg.robot.type == "cobot_magic_ros_follower"
    assert cfg.reset_to_zero_pose is True
    assert cfg.reset_before_replay is True
    assert cfg.reset_after_replay is True
    assert cfg.require_replay_start_pose is True
    assert cfg.reset_duration_s == pytest.approx(6.0)
    assert cfg.reset_pose_tolerance == pytest.approx(0.02)


def test_cobot_magic_ros_default_camera_topics_match_runtime():
    cfg = CobotMagicRosFollowerConfig(id="ros_follower")

    assert cfg.cameras["cam_high"].topic == "/camera_f/color/image_raw"
    assert cfg.cameras["cam_left_wrist"].topic == "/camera_l/color/image_raw"
    assert cfg.cameras["cam_right_wrist"].topic == "/camera_r/color/image_raw"


def test_cobot_magic_ros_zero_pose_uses_fixed_project_pose():
    cfg = SimpleNamespace(
        robot=SimpleNamespace(type="cobot_magic_ros_follower", id="ros_follower"),
        reset_pose_path=None,
        reset_to_zero_pose=True,
        capture_reset_pose=False,
        reset_before_record=False,
        reset_after_episode=False,
    )

    reset_pose = _prepare_record_reset_pose(cfg, robot=None)

    assert reset_pose["left_joint_1.pos"] == pytest.approx(-0.00209808349609375)
    assert reset_pose["right_joint_6.pos"] == pytest.approx(-0.00362396240234375)


def test_cobot_magic_ros_leader_start_check_uses_saved_leader_pose(tmp_path):
    pose_path = tmp_path / "fixed_pose.json"
    pose_path.write_text(
        json.dumps(
            {
                "joint_pos": {"left_joint_1.pos": 0.0, "right_joint_1.pos": 0.0},
                "leader_joint_pos": {"left_joint_1.pos": 0.25, "right_joint_1.pos": -0.4},
            }
        )
    )
    cfg = SimpleNamespace(
        reset_to_zero_pose=True,
        reset_pose_tolerance=0.05,
        reset_pose_path=pose_path,
    )
    reset_pose = {"left_joint_1.pos": 0.0, "right_joint_1.pos": 0.0}
    aligned_teleop = SimpleNamespace(
        get_absolute_action=lambda: {"left_joint_1.pos": 0.26, "right_joint_1.pos": -0.42}
    )
    _assert_teleop_matches_reset_pose_if_required(
        cfg=cfg,
        teleop=aligned_teleop,
        reset_pose=reset_pose,
    )

    misaligned_teleop = SimpleNamespace(
        get_absolute_action=lambda: {"left_joint_1.pos": 0.25, "right_joint_1.pos": -0.6}
    )
    with pytest.raises(RuntimeError, match="leader is not aligned"):
        _assert_teleop_matches_reset_pose_if_required(
            cfg=cfg,
            teleop=misaligned_teleop,
            reset_pose=reset_pose,
        )


def test_cobot_magic_ros_replay_zero_pose_uses_first_action():
    dataset = SimpleNamespace(features={ACTION: {"names": ["left_joint_1.pos", "left_joint_1.vel"]}})
    actions = [{ACTION: [0.35, 9.0]}]

    reset_pose = _first_replay_action_pose(dataset, actions)

    assert reset_pose == {"left_joint_1.pos": 0.35}


def test_cobot_magic_ros_robot_publishes_jointstate_and_reads_observation(monkeypatch):
    fake_rospy = install_fake_ros(monkeypatch)
    robot = make_robot_from_config(CobotMagicRosFollowerConfig(id="ros_follower", cameras={}))
    robot.connect()
    try:
        fake_rospy.subscribers["/cobot_magic/puppet/joint_left"][0].emit(
            make_joint_state([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], effort=[1, 2, 3, 4, 5, 6, 7])
        )
        fake_rospy.subscribers["/cobot_magic/puppet/joint_right"][0].emit(
            make_joint_state([-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7])
        )

        obs = robot.get_observation()
        assert obs["left_joint_1.pos"] == pytest.approx(0.1)
        assert obs["left_gripper.torque"] == pytest.approx(7.0)
        assert obs["right_joint_6.pos"] == pytest.approx(-0.6)

        action = {
            **{f"left_joint_{idx}.pos": float(idx) for idx in range(1, 7)},
            "left_gripper.pos": 7.0,
            **{f"right_joint_{idx}.pos": -float(idx) for idx in range(1, 7)},
            "right_gripper.pos": -7.0,
        }
        sent = robot.send_action(action)

        left_msg = fake_rospy.publishers["/cobot_magic/command/joint_left"].published[-1]
        right_msg = fake_rospy.publishers["/cobot_magic/command/joint_right"].published[-1]
        assert left_msg.name == ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        assert left_msg.position == pytest.approx([1, 2, 3, 4, 5, 6, 7])
        assert right_msg.position == pytest.approx([-1, -2, -3, -4, -5, -6, -7])
        assert sent["left_joint_1.pos"] == pytest.approx(1.0)
    finally:
        robot.disconnect()


def test_cobot_magic_ros_leader_relative_takeover(monkeypatch):
    fake_rospy = install_fake_ros(monkeypatch)
    teleop = make_teleoperator_from_config(CobotMagicRosLeaderConfig(id="ros_leader"))
    teleop.connect()
    try:
        fake_rospy.subscribers["/cobot_magic/leader/joint_left"][0].emit(
            make_joint_state([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        )
        fake_rospy.subscribers["/cobot_magic/leader/joint_right"][0].emit(
            make_joint_state([-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7])
        )
        fake_rospy.subscribers["/cobot_magic/puppet/joint_left"][0].emit(
            make_joint_state([1.0, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7])
        )
        fake_rospy.subscribers["/cobot_magic/puppet/joint_right"][0].emit(
            make_joint_state([-1.0, -1.2, -1.3, -1.4, -1.5, -1.6, -1.7])
        )

        teleop.set_manual_control(True)
        fake_rospy.subscribers["/cobot_magic/leader/joint_left"][0].emit(
            make_joint_state([0.3, 0.2, 0.3, 0.4, 0.5, 0.4, 0.8])
        )

        action = teleop.get_action()
        assert action["left_joint_1.pos"] == pytest.approx(1.2)
        assert action["left_joint_6.pos"] == pytest.approx(1.4)
        assert action["left_gripper.pos"] == pytest.approx(1.8)
        assert action["right_joint_1.pos"] == pytest.approx(-1.0)
    finally:
        teleop.disconnect()


def test_cobot_magic_ros_leader_feedback_commands_and_manual_mode(monkeypatch):
    fake_rospy = install_fake_ros(monkeypatch)
    teleop = make_teleoperator_from_config(CobotMagicRosLeaderConfig(id="ros_leader"))
    teleop.connect()
    try:
        assert fake_rospy.publishers["/cobot_magic/leader/manual_control_left"].published[-1].data is False
        assert fake_rospy.publishers["/cobot_magic/leader/manual_control_right"].published[-1].data is False

        emit_topic(fake_rospy, "/cobot_magic/leader/joint_left", make_joint_state([0.0] * 7))
        emit_topic(fake_rospy, "/cobot_magic/leader/joint_right", make_joint_state([0.0] * 7))
        emit_topic(fake_rospy, "/cobot_magic/puppet/joint_left", make_joint_state([0.0] * 7))
        emit_topic(fake_rospy, "/cobot_magic/puppet/joint_right", make_joint_state([0.0] * 7))
        teleop.set_manual_control(True)
        assert fake_rospy.publishers["/cobot_magic/leader/manual_control_left"].published[-1].data is True
        assert fake_rospy.publishers["/cobot_magic/leader/manual_control_right"].published[-1].data is True

        feedback = {
            **{f"left_joint_{idx}.pos": float(idx) for idx in range(1, 7)},
            "left_gripper.pos": 0.7,
            **{f"right_joint_{idx}.pos": -float(idx) for idx in range(1, 7)},
            "right_gripper.pos": 0.8,
        }
        teleop.send_feedback(feedback)

        left_msg = fake_rospy.publishers["/cobot_magic/leader/command_joint_left"].published[-1]
        right_msg = fake_rospy.publishers["/cobot_magic/leader/command_joint_right"].published[-1]
        assert fake_rospy.publishers["/cobot_magic/leader/manual_control_left"].published[-1].data is False
        assert left_msg.position == pytest.approx([1, 2, 3, 4, 5, 6, 0.7])
        assert right_msg.position == pytest.approx([-1, -2, -3, -4, -5, -6, 0.8])
    finally:
        teleop.disconnect()


def test_cobot_magic_ros_startup_sync_aligns_to_absolute_leader(monkeypatch):
    fake_rospy = install_fake_ros(monkeypatch)
    robot = make_robot_from_config(
        CobotMagicRosFollowerConfig(id="ros_follower", cameras={}, sync_gripper=False)
    )
    teleop = make_teleoperator_from_config(
        CobotMagicRosLeaderConfig(
            id="ros_leader",
            sync_gripper=False,
            manual_control=False,
            startup_sync=True,
            startup_sync_duration_s=0.01,
            startup_sync_max_joint_delta=None,
        )
    )
    teleop.connect()
    robot.connect()
    try:
        emit_topic(
            fake_rospy,
            "/cobot_magic/leader/joint_left",
            make_joint_state([1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 0.0]),
        )
        emit_topic(
            fake_rospy,
            "/cobot_magic/leader/joint_right",
            make_joint_state([-1.0, -1.1, -1.2, -1.3, -1.4, -1.5, 0.0]),
        )
        emit_topic(fake_rospy, "/cobot_magic/puppet/joint_left", make_joint_state([0.0] * 7))
        emit_topic(fake_rospy, "/cobot_magic/puppet/joint_right", make_joint_state([0.0] * 7))
        teleop.config.manual_control = True

        send_action = robot.send_action

        def send_action_and_update_state(action):
            sent = send_action(action)
            left_msg = fake_rospy.publishers["/cobot_magic/command/joint_left"].published[-1]
            right_msg = fake_rospy.publishers["/cobot_magic/command/joint_right"].published[-1]
            emit_topic(fake_rospy, "/cobot_magic/puppet/joint_left", make_joint_state(left_msg.position))
            emit_topic(fake_rospy, "/cobot_magic/puppet/joint_right", make_joint_state(right_msg.position))
            return sent

        robot.send_action = send_action_and_update_state

        _run_startup_sync_if_requested(robot, teleop, fps=100)

        left_msg = fake_rospy.publishers["/cobot_magic/command/joint_left"].published[-1]
        right_msg = fake_rospy.publishers["/cobot_magic/command/joint_right"].published[-1]
        assert left_msg.position[:6] == pytest.approx([1.0, 1.1, 1.2, 1.3, 1.4, 1.5])
        assert right_msg.position[:6] == pytest.approx([-1.0, -1.1, -1.2, -1.3, -1.4, -1.5])

        action = teleop.get_action()
        assert action["left_joint_1.pos"] == pytest.approx(1.0)
        assert action["right_joint_6.pos"] == pytest.approx(-1.5)
    finally:
        teleop.disconnect()
        robot.disconnect()


def test_cobot_magic_ros_robot_converts_bgr_image(monkeypatch):
    fake_rospy = install_fake_ros(monkeypatch)
    robot = make_robot_from_config(
        CobotMagicRosFollowerConfig(
            id="ros_follower",
            cameras={
                "cam_high": CobotMagicRosCameraConfig(
                    topic="/camera/color/image_raw",
                    height=1,
                    width=2,
                )
            },
        )
    )
    robot.connect()
    try:
        fake_rospy.subscribers["/cobot_magic/puppet/joint_left"][0].emit(make_joint_state([0] * 7))
        fake_rospy.subscribers["/cobot_magic/puppet/joint_right"][0].emit(make_joint_state([0] * 7))
        bgr = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
        fake_rospy.subscribers["/camera/color/image_raw"][0].emit(
            FakeImage(data=bgr.tobytes(), height=1, width=2, encoding="bgr8")
        )

        obs = robot.get_observation()
        assert obs["cam_high"].tolist() == [[[3, 2, 1], [6, 5, 4]]]
    finally:
        robot.disconnect()
