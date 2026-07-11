from types import SimpleNamespace

import pytest

from lerobot.teleoperators.cobot_magic_ros import cobot_magic_ros as leader_module
from lerobot.teleoperators.cobot_magic_ros.cobot_magic_ros import CobotMagicRosLeader
from lerobot.teleoperators.cobot_magic_ros.config_cobot_magic_ros import CobotMagicRosLeaderConfig


class _Publisher:
    def __init__(self, on_publish=None) -> None:
        self.messages = []
        self._on_publish = on_publish

    def publish(self, message) -> None:
        self.messages.append(message)
        if self._on_publish is not None:
            self._on_publish(message)


def test_leader_callbacks_record_monotonic_receive_time(monkeypatch):
    leader = CobotMagicRosLeader.__new__(CobotMagicRosLeader)
    leader._left_leader_state = None
    leader._left_leader_state_received_at = None
    message = object()
    monkeypatch.setattr(leader_module.time, "monotonic", lambda: 12.5)

    leader._left_leader_state_callback(message)

    assert leader._left_leader_state is message
    assert leader._left_leader_state_received_at == 12.5


def test_wait_for_processes_callbacks_when_cached_state_is_stale(monkeypatch):
    leader = CobotMagicRosLeader.__new__(CobotMagicRosLeader)
    leader.config = SimpleNamespace(read_timeout_s=0.1, poll_interval_s=0.001, state_timeout_s=0.25)
    leader._ros = object()
    leader._left_leader_state = object()
    leader._left_leader_state_received_at = 1.0
    spin_calls = []
    monkeypatch.setattr(leader_module.time, "monotonic", lambda: 10.0)
    monkeypatch.setattr(leader_module.time, "sleep", lambda _seconds: None)

    def spin(_ros, *, timeout_sec):
        spin_calls.append(timeout_sec)
        leader._left_leader_state_received_at = 10.0

    monkeypatch.setattr(leader_module, "spin_ros_once", spin)

    leader._wait_for(
        lambda: leader._state_is_fresh(
            leader._left_leader_state,
            leader._left_leader_state_received_at,
        )
    )

    assert spin_calls == [0.0]


def test_send_feedback_switches_manual_state_via_command_only(monkeypatch):
    leader = CobotMagicRosLeader.__new__(CobotMagicRosLeader)
    leader._is_connected = True
    leader._ros = object()
    leader.config = SimpleNamespace(sync_gripper=True, read_timeout_s=0.0, poll_interval_s=0.01)
    leader._manual_control_enabled = True
    leader._leader_origin = {"left_joint_1.pos": 0.0}
    leader._follower_origin = {"left_joint_1.pos": 0.0}
    leader._left_leader_state = object()
    leader._right_leader_state = object()
    leader._left_command_publisher = _Publisher(
        lambda _message: setattr(leader, "_left_manual_control_state", False)
    )
    leader._right_command_publisher = _Publisher(
        lambda _message: setattr(leader, "_right_manual_control_state", False)
    )
    leader._left_manual_control_state = True
    leader._right_manual_control_state = True

    monkeypatch.setattr(
        leader_module,
        "build_ros_joint_positions",
        lambda _feedback, prefix, **_kwargs: ([1.0], {f"{prefix}_joint_1.pos": 1.0}),
    )
    monkeypatch.setattr(leader_module, "make_joint_state_message", lambda _ros, positions: positions)
    monkeypatch.setattr(
        leader,
        "set_manual_control",
        lambda _enabled: (_ for _ in ()).throw(AssertionError("manual topic must not be used")),
    )

    leader.send_feedback({"left_joint_1.pos": 1.0, "right_joint_1.pos": 1.0})

    assert leader._manual_control_enabled is False
    assert leader._leader_origin is None
    assert leader._follower_origin is None
    assert leader._left_command_publisher.messages == [[1.0]]
    assert leader._right_command_publisher.messages == [[1.0]]


def test_manual_control_request_clears_cached_ack_before_publish():
    leader = CobotMagicRosLeader.__new__(CobotMagicRosLeader)
    leader.config = SimpleNamespace(read_timeout_s=0.0, poll_interval_s=0.01)
    leader._ros = None
    leader._left_manual_control_state = True
    leader._right_manual_control_state = True
    states_seen_during_publish = []

    def publish(enabled):
        states_seen_during_publish.append(
            (enabled, leader._left_manual_control_state, leader._right_manual_control_state)
        )
        leader._left_manual_control_state = enabled
        leader._right_manual_control_state = enabled

    leader._publish_manual_control = publish

    leader._request_manual_control(False)

    assert states_seen_during_publish == [(False, None, None)]


def test_manual_control_ack_requires_both_arms():
    leader = CobotMagicRosLeader.__new__(CobotMagicRosLeader)
    leader.config = SimpleNamespace(read_timeout_s=0.0, poll_interval_s=0.01)
    leader._ros = None
    leader._left_manual_control_state = True
    leader._right_manual_control_state = False

    try:
        leader._wait_for_manual_control_state(True)
    except RuntimeError as exc:
        assert "requested=True left=True right=False" in str(exc)
    else:
        raise AssertionError("mismatched leader modes must not be accepted")


def test_manual_control_validates_relative_origins_before_hardware_transition():
    leader = CobotMagicRosLeader.__new__(CobotMagicRosLeader)
    leader._is_connected = True
    leader.config = SimpleNamespace(relative_takeover=True)
    requests = []
    leader._absolute_leader_action = lambda: (_ for _ in ()).throw(RuntimeError("stale leader"))
    leader._follower_action = lambda: {"state": 1.0}
    leader._request_manual_control = lambda enabled: requests.append(enabled)

    try:
        leader.set_manual_control(True)
    except RuntimeError as exc:
        assert "stale leader" in str(exc)
    else:
        raise AssertionError("stale relative origin must reject takeover")

    assert requests == []


def test_manual_control_ack_failure_rolls_back_both_arms():
    leader = CobotMagicRosLeader.__new__(CobotMagicRosLeader)
    leader._is_connected = True
    leader.config = SimpleNamespace(relative_takeover=False)
    leader._manual_control_enabled = False
    leader._leader_origin = None
    leader._follower_origin = None
    requests = []

    def request(enabled):
        requests.append(enabled)
        if enabled:
            raise RuntimeError("right arm did not acknowledge")

    leader._request_manual_control = request

    try:
        leader.set_manual_control(True)
    except RuntimeError as exc:
        assert "right arm" in str(exc)
    else:
        raise AssertionError("failed mode ACK must reject takeover")

    assert requests == [True, False]
    assert leader._manual_control_enabled is False


def test_send_feedback_build_failure_keeps_manual_state(monkeypatch):
    leader = CobotMagicRosLeader.__new__(CobotMagicRosLeader)
    leader._is_connected = True
    leader._ros = object()
    leader.config = SimpleNamespace(sync_gripper=True)
    leader._manual_control_enabled = True
    leader._leader_origin = {"left_joint_1.pos": 0.0}
    leader._follower_origin = {"left_joint_1.pos": 0.0}
    leader._left_leader_state = object()
    leader._right_leader_state = object()
    monkeypatch.setattr(
        leader_module,
        "build_ros_joint_positions",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(KeyError("missing joint")),
    )

    try:
        leader.send_feedback({})
    except KeyError as exc:
        assert "missing joint" in str(exc)
    else:
        raise AssertionError("invalid feedback must fail")

    assert leader._manual_control_enabled is True
    assert leader._leader_origin is not None


def test_send_feedback_rejects_cached_false_without_fresh_arm_acks(monkeypatch):
    leader = CobotMagicRosLeader.__new__(CobotMagicRosLeader)
    leader._is_connected = True
    leader._ros = object()
    leader.config = SimpleNamespace(sync_gripper=True, read_timeout_s=0.0, poll_interval_s=0.01)
    leader._manual_control_enabled = True
    leader._leader_origin = {"left_joint_1.pos": 0.0}
    leader._follower_origin = {"left_joint_1.pos": 0.0}
    leader._left_leader_state = object()
    leader._right_leader_state = object()
    leader._left_command_publisher = _Publisher()
    leader._right_command_publisher = _Publisher()
    leader._left_manual_control_state = False
    leader._right_manual_control_state = False
    rollback_calls = []
    leader._best_effort_disable_manual_control = lambda: rollback_calls.append(True) or False
    monkeypatch.setattr(
        leader_module,
        "build_ros_joint_positions",
        lambda _feedback, prefix, **_kwargs: ([1.0], {f"{prefix}_joint_1.pos": 1.0}),
    )
    monkeypatch.setattr(leader_module, "make_joint_state_message", lambda _ros, positions: positions)

    with pytest.raises(RuntimeError, match=r"requested=False left=None right=None"):
        leader.send_feedback({"left_joint_1.pos": 1.0, "right_joint_1.pos": 1.0})

    assert rollback_calls == [True]
    assert leader._manual_control_enabled is True


def test_leader_rejects_stale_ros_state(monkeypatch):
    leader = CobotMagicRosLeader.__new__(CobotMagicRosLeader)
    leader.config = SimpleNamespace(state_timeout_s=0.25)
    monkeypatch.setattr(leader_module.time, "monotonic", lambda: 10.0)

    try:
        leader._require_state(object(), 9.0, "/leader/right")
    except RuntimeError as exc:
        assert "Stale" in str(exc)
        assert "age=1.000s" in str(exc)
    else:
        raise AssertionError("stale leader state must not be accepted")


def test_disconnect_disables_manual_control_before_destroying_ros_entities(tmp_path):
    leader = CobotMagicRosLeader(CobotMagicRosLeaderConfig(id="disconnect-test", calibration_dir=tmp_path))
    events = []

    class Node:
        def destroy_subscription(self, subscriber):
            events.append(("destroy_subscription", subscriber))

        def destroy_publisher(self, publisher):
            events.append(("destroy_publisher", publisher))

    leader._ros_node = Node()
    leader._subscribers = ["subscriber"]
    leader._left_command_publisher = "left_command"
    leader._right_command_publisher = "right_command"
    leader._left_manual_control_publisher = "left_manual"
    leader._right_manual_control_publisher = "right_manual"
    leader._is_connected = True
    leader._manual_control_enabled = True
    leader._left_manual_control_state = True
    leader._right_manual_control_state = True
    leader._left_leader_state = object()
    leader._left_leader_state_received_at = 1.0
    leader._request_manual_control = lambda enabled: events.append(("request", enabled))

    leader.disconnect()

    assert events[0] == ("request", False)
    assert events[1:] == [
        ("destroy_subscription", "subscriber"),
        ("destroy_publisher", "left_command"),
        ("destroy_publisher", "right_command"),
        ("destroy_publisher", "left_manual"),
        ("destroy_publisher", "right_manual"),
    ]
    assert leader.is_connected is False
    assert leader._left_leader_state is None
    assert leader._left_leader_state_received_at is None
    assert leader._left_manual_control_state is None
    assert leader._right_manual_control_state is None


def test_connect_failure_cleans_up_partially_created_ros_entities(monkeypatch, tmp_path):
    leader = CobotMagicRosLeader(
        CobotMagicRosLeaderConfig(id="connect-failure-test", calibration_dir=tmp_path)
    )

    class Node:
        def __init__(self):
            self.publishers = []
            self.subscribers = []
            self.destroyed_publishers = []
            self.destroyed_subscribers = []

        def create_publisher(self, _msg_type, topic, _queue_size):
            publisher = f"publisher:{topic}"
            self.publishers.append(publisher)
            return publisher

        def create_subscription(self, _msg_type, topic, _callback, _queue_size):
            if len(self.subscribers) == 1:
                raise RuntimeError("subscription setup failed")
            subscriber = f"subscriber:{topic}"
            self.subscribers.append(subscriber)
            return subscriber

        def destroy_publisher(self, publisher):
            self.destroyed_publishers.append(publisher)

        def destroy_subscription(self, subscriber):
            self.destroyed_subscribers.append(subscriber)

    node = Node()
    fake_ros = SimpleNamespace(JointState=object, PoseStamped=object, Bool=object)
    monkeypatch.setattr(leader_module, "import_ros", lambda: fake_ros)
    monkeypatch.setattr(leader_module, "ensure_ros_node", lambda _ros, _name: node)

    with pytest.raises(RuntimeError, match="subscription setup failed"):
        leader.connect()

    assert leader.is_connected is False
    assert node.destroyed_publishers == node.publishers
    assert node.destroyed_subscribers == node.subscribers
