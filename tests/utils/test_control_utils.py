from types import SimpleNamespace

import pytest

from lerobot.utils.control_utils import sanity_check_bimanual_piper_pair


@pytest.mark.parametrize(
    ("robot_type", "teleop_type"),
    [
        ("bi_piper_follower", "bi_piper_leader"),
        ("bi_piperx_follower", "bi_piperx_leader"),
        ("cobot_magic_follower", "cobot_magic_leader"),
        ("cobot_magic_ros_follower", "cobot_magic_ros_leader"),
        ("so101_follower", "so101_leader"),
    ],
)
def test_sanity_check_bimanual_piper_pair_accepts_valid_pairs(robot_type, teleop_type):
    sanity_check_bimanual_piper_pair(
        SimpleNamespace(type=robot_type),
        SimpleNamespace(type=teleop_type),
    )


def test_sanity_check_bimanual_piper_pair_accepts_missing_teleop():
    sanity_check_bimanual_piper_pair(SimpleNamespace(type="bi_piperx_follower"), None)


def test_sanity_check_cobot_magic_rejects_shared_leader_follower_interface():
    robot_cfg = SimpleNamespace(
        type="cobot_magic_follower",
        left_arm_config=SimpleNamespace(interface="can0"),
        right_arm_config=SimpleNamespace(interface="can1"),
    )
    teleop_cfg = SimpleNamespace(
        type="cobot_magic_leader",
        left_arm_config=SimpleNamespace(interface="can0"),
        right_arm_config=SimpleNamespace(interface="can3"),
    )

    with pytest.raises(ValueError, match="interfaces must be unique"):
        sanity_check_bimanual_piper_pair(robot_cfg, teleop_cfg)


@pytest.mark.parametrize(
    ("robot_type", "teleop_type"),
    [
        ("bi_piper_follower", "bi_piperx_leader"),
        ("bi_piperx_follower", "bi_piper_leader"),
        ("cobot_magic_follower", "bi_piper_leader"),
        ("cobot_magic_ros_follower", "cobot_magic_leader"),
        ("cobot_magic_follower", "cobot_magic_ros_leader"),
        ("bi_piperx_follower", "cobot_magic_leader"),
        ("so101_follower", "bi_piperx_leader"),
        ("so101_follower", "bi_piper_leader"),
        ("so101_follower", "cobot_magic_leader"),
    ],
)
def test_sanity_check_bimanual_piper_pair_rejects_mixed_pairs(robot_type, teleop_type):
    with pytest.raises(ValueError, match="must be paired"):
        sanity_check_bimanual_piper_pair(
            SimpleNamespace(type=robot_type),
            SimpleNamespace(type=teleop_type),
        )
