import pytest

from lerobot.scripts.recording_loop import (
    _park_teleop_at_current_pose,
    _set_teleop_manual_control,
)


def test_manual_control_switch_failure_is_not_swallowed():
    class Teleop:
        def set_manual_control(self, enabled):
            raise RuntimeError(f"mode ACK failed: {enabled}")

    with pytest.raises(RuntimeError, match="mode ACK failed: True"):
        _set_teleop_manual_control(Teleop(), True)


def test_manual_control_switch_is_optional_for_other_teleoperators():
    _set_teleop_manual_control(object(), True)


def test_parked_release_switches_with_current_position_command_only():
    class Teleop:
        def __init__(self):
            self.feedback = []
            self.manual_requests = []

        def get_absolute_action(self):
            return {"left_joint_1.pos": 0.25}

        def send_feedback(self, action):
            self.feedback.append(dict(action))

        def set_manual_control(self, enabled):
            self.manual_requests.append(enabled)

    teleop = Teleop()

    _park_teleop_at_current_pose(teleop)

    assert teleop.feedback == [{"left_joint_1.pos": 0.25}]
    assert teleop.manual_requests == []
