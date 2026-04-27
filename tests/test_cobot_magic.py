import sys
from types import SimpleNamespace

import draccus
import numpy as np
import pytest
from draccus.utils import ParsingError

from lerobot.processor import make_default_processors
from lerobot.robots.cobot_magic import CobotMagicFollower, CobotMagicFollowerConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.scripts.lerobot_record import RecordConfig
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig
from lerobot.teleoperators.cobot_magic import CobotMagicLeader, CobotMagicLeaderConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.utils.arx5_sdk import CobotMagicArmConfig


class FakeLogLevel:
    TRACE = "TRACE"
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"
    OFF = "OFF"


class FakeRobotConfig:
    def __init__(self, model: str):
        self.robot_model = model
        self.joint_dof = 6
        self.joint_pos_min = np.array([-3.14] * 6)
        self.joint_pos_max = np.array([3.14] * 6)
        self.joint_vel_max = np.array([2.0] * 6)
        self.joint_torque_max = np.array([15.0] * 6)
        self.gripper_width = 0.08


class FakeControllerConfig:
    def __init__(self):
        self.controller_type = "joint_controller"
        self.controller_dt = 0.002
        self.gravity_compensation = False
        self.background_send_recv = False
        self.shutdown_to_passive = False


class FakeRobotConfigFactory:
    @classmethod
    def get_instance(cls):
        return cls()

    def get_config(self, robot_model: str):
        return FakeRobotConfig(robot_model)


class FakeControllerConfigFactory:
    @classmethod
    def get_instance(cls):
        return cls()

    def get_config(self, controller_type: str, joint_dof: int):
        del controller_type, joint_dof
        return FakeControllerConfig()


class FakeJointState:
    def __init__(
        self,
        dof_or_pos,
        vel=None,
        torque=None,
        gripper_pos=0.0,
        gripper_vel=0.0,
        gripper_torque=0.0,
    ):
        if isinstance(dof_or_pos, int):
            self._pos = np.zeros(dof_or_pos, dtype=np.float64)
            self._vel = np.zeros(dof_or_pos, dtype=np.float64)
            self._torque = np.zeros(dof_or_pos, dtype=np.float64)
        else:
            self._pos = np.asarray(dof_or_pos, dtype=np.float64).copy()
            self._vel = np.asarray(vel, dtype=np.float64).copy()
            self._torque = np.asarray(torque, dtype=np.float64).copy()
        self.timestamp = 1.0
        self.gripper_pos = float(gripper_pos)
        self.gripper_vel = float(gripper_vel)
        self.gripper_torque = float(gripper_torque)

    def pos(self):
        return self._pos

    def vel(self):
        return self._vel

    def torque(self):
        return self._torque


class FakeArx5JointController:
    def __init__(self, robot_config, controller_config, interface):
        self.robot_config = robot_config
        self.controller_config = controller_config
        self.interface = interface
        self.log_level = None
        self.damping_calls = 0
        self.home_calls = 0
        self.send_recv_calls = 0
        self.recv_calls = 0
        self.last_cmd = None
        self.state = FakeJointState(
            np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]),
            np.array([0.01, 0.02, 0.03, 0.04, 0.05, 0.06]),
            np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]),
            0.03,
            0.001,
            0.2,
        )

    def send_recv_once(self):
        self.send_recv_calls += 1

    def recv_once(self):
        self.recv_calls += 1

    def set_joint_cmd(self, cmd):
        self.last_cmd = cmd
        self.state = FakeJointState(cmd.pos().copy(), np.zeros(6), np.zeros(6), cmd.gripper_pos)

    def get_joint_cmd(self):
        return self.last_cmd

    def get_joint_state(self):
        return self.state

    def get_robot_config(self):
        return self.robot_config

    def get_controller_config(self):
        return self.controller_config

    def reset_to_home(self):
        self.home_calls += 1

    def set_to_damping(self):
        self.damping_calls += 1

    def set_log_level(self, level):
        self.log_level = level


def install_fake_arx5(monkeypatch):
    fake_module = SimpleNamespace(
        Arx5JointController=FakeArx5JointController,
        ControllerConfigFactory=FakeControllerConfigFactory,
        JointState=FakeJointState,
        LogLevel=FakeLogLevel,
        RobotConfigFactory=FakeRobotConfigFactory,
    )
    monkeypatch.setitem(sys.modules, "arx5_interface", fake_module)
    return fake_module


def make_arm(interface: str, **kwargs) -> CobotMagicArmConfig:
    return CobotMagicArmConfig(interface=interface, **kwargs)


def make_robot_config(**kwargs) -> CobotMagicFollowerConfig:
    left_arm_config = kwargs.pop("left_arm_config", make_arm("can0"))
    right_arm_config = kwargs.pop("right_arm_config", make_arm("can1"))
    return CobotMagicFollowerConfig(
        left_arm_config=left_arm_config,
        right_arm_config=right_arm_config,
        **kwargs,
    )


def make_leader_config(**kwargs) -> CobotMagicLeaderConfig:
    left_arm_config = kwargs.pop("left_arm_config", make_arm("can2"))
    right_arm_config = kwargs.pop("right_arm_config", make_arm("can3"))
    return CobotMagicLeaderConfig(
        left_arm_config=left_arm_config,
        right_arm_config=right_arm_config,
        **kwargs,
    )


def test_cobot_magic_parse_teleoperate_config():
    args = [
        "--robot.type=cobot_magic_follower",
        "--robot.left_arm_config.interface=can0",
        "--robot.right_arm_config.interface=can1",
        "--teleop.type=cobot_magic_leader",
        "--teleop.left_arm_config.interface=can2",
        "--teleop.right_arm_config.interface=can3",
    ]

    cfg = draccus.parse(config_class=TeleoperateConfig, config_path=None, args=args)

    assert cfg.robot.type == "cobot_magic_follower"
    assert cfg.teleop.type == "cobot_magic_leader"
    assert cfg.robot.left_arm_config.model == "X5"
    assert cfg.teleop.left_arm_config.gravity_compensation is True


def test_cobot_magic_record_rejects_mismatched_pair():
    args = [
        "--robot.type=cobot_magic_follower",
        "--robot.left_arm_config.interface=can0",
        "--robot.right_arm_config.interface=can1",
        "--teleop.type=so101_leader",
        "--teleop.port=/dev/mock",
        "--dataset.repo_id=dummy/dummy",
        "--dataset.single_task=test",
        "--dataset.num_episodes=1",
        "--dataset.episode_time_s=1",
        "--dataset.reset_time_s=1",
        "--dataset.push_to_hub=false",
    ]
    with pytest.raises(ParsingError) as exc_info:
        draccus.parse(config_class=RecordConfig, config_path=None, args=args)
    assert exc_info.value.__cause__ is not None
    assert "must be paired" in str(exc_info.value.__cause__)


def test_cobot_magic_factories_and_features_without_sdk():
    robot = make_robot_from_config(make_robot_config())
    teleop = make_teleoperator_from_config(make_leader_config())

    assert isinstance(robot, CobotMagicFollower)
    assert isinstance(teleop, CobotMagicLeader)
    assert robot.action_features["left_joint_1.pos"] is float
    assert robot.observation_features["right_joint_6.torque"] is float
    assert teleop.action_features["left_gripper.pos"] is float


def test_cobot_magic_leader_manual_mode_enables_damping_and_gravity_comp(monkeypatch):
    install_fake_arx5(monkeypatch)

    teleop = make_teleoperator_from_config(make_leader_config())
    teleop.connect()
    try:
        assert teleop.left_arm.controller.controller_config.gravity_compensation is True
        assert teleop.left_arm.controller.damping_calls == 1
        action = teleop.get_action()
        assert action["left_joint_1.pos"] == pytest.approx(0.1)
        assert action["right_gripper.pos"] == pytest.approx(0.03)
    finally:
        teleop.disconnect()


def test_cobot_magic_follower_clamps_and_sends_full_joint_state(monkeypatch):
    install_fake_arx5(monkeypatch)

    robot = make_robot_from_config(
        make_robot_config(
            left_arm_config=make_arm("can0", max_relative_target=0.05),
            right_arm_config=make_arm("can1", max_relative_target=0.05),
        )
    )
    robot.connect()
    try:
        assert "left_gripper.pos" not in robot.action_features
        action = {
            "left_joint_1.pos": 1.0,
            "left_joint_2.pos": 1.0,
            "left_joint_3.pos": 1.0,
            "left_joint_4.pos": 1.0,
            "left_joint_5.pos": 1.0,
            "left_joint_6.pos": 1.0,
            "left_gripper.pos": 0.08,
            "right_joint_1.pos": -1.0,
            "right_joint_2.pos": -1.0,
            "right_joint_3.pos": -1.0,
            "right_joint_4.pos": -1.0,
            "right_joint_5.pos": -1.0,
            "right_joint_6.pos": -1.0,
            "right_gripper.pos": 0.0,
        }
        sent = robot.send_action(action)

        assert sent["left_joint_1.pos"] == pytest.approx(0.15)
        assert sent["right_joint_1.pos"] == pytest.approx(0.05)
        assert robot.left_arm.controller.last_cmd.pos()[0] == pytest.approx(0.15)
        assert robot.right_arm.controller.last_cmd.gripper_pos == pytest.approx(0.0)
    finally:
        robot.disconnect()


def test_cobot_magic_follower_sync_gripper_false_holds_current_gripper(monkeypatch):
    install_fake_arx5(monkeypatch)

    robot = make_robot_from_config(
        make_robot_config(
            left_arm_config=make_arm("can0", sync_gripper=False),
            right_arm_config=make_arm("can1", sync_gripper=False),
        )
    )
    robot.connect()
    try:
        action = {
            "left_joint_1.pos": 0.11,
            "left_joint_2.pos": 0.21,
            "left_joint_3.pos": 0.31,
            "left_joint_4.pos": 0.41,
            "left_joint_5.pos": 0.51,
            "left_joint_6.pos": 0.61,
            "left_gripper.pos": 0.0,
            "right_joint_1.pos": 0.11,
            "right_joint_2.pos": 0.21,
            "right_joint_3.pos": 0.31,
            "right_joint_4.pos": 0.41,
            "right_joint_5.pos": 0.51,
            "right_joint_6.pos": 0.61,
            "right_gripper.pos": 0.0,
        }
        robot.send_action(action)

        assert robot.left_arm.controller.last_cmd.gripper_pos == pytest.approx(0.03)
        assert robot.right_arm.controller.last_cmd.gripper_pos == pytest.approx(0.03)
    finally:
        robot.disconnect()


def test_cobot_magic_leader_sync_gripper_false_omits_gripper(monkeypatch):
    install_fake_arx5(monkeypatch)

    teleop = make_teleoperator_from_config(
        make_leader_config(
            left_arm_config=make_arm("can2", sync_gripper=False),
            right_arm_config=make_arm("can3", sync_gripper=False),
        )
    )
    teleop.connect()
    try:
        assert "left_gripper.pos" not in teleop.action_features
        action = teleop.get_action()

        assert "left_gripper.pos" not in action
        assert "right_gripper.pos" not in action
        assert action["left_joint_1.pos"] == pytest.approx(0.1)
    finally:
        teleop.disconnect()


def test_cobot_magic_roundtrip_through_default_processors(monkeypatch):
    install_fake_arx5(monkeypatch)

    teleop = make_teleoperator_from_config(make_leader_config())
    robot = make_robot_from_config(make_robot_config())
    teleop_action_processor, robot_action_processor, _ = make_default_processors()

    teleop.connect()
    robot.connect()
    try:
        action = teleop_action_processor(teleop.get_action())
        obs = robot.get_observation()
        processed_action = robot_action_processor((action, obs))
        sent = robot.send_action(processed_action)
        assert sent["left_joint_1.pos"] == pytest.approx(0.1)
        assert sent["right_joint_6.pos"] == pytest.approx(0.6)
    finally:
        teleop.disconnect()
        robot.disconnect()
