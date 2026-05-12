import numpy as np

from lerobot.scripts.convert_cobot_magic_open_drawer import (
    ACTION_NAMES,
    CAMERA_KEY,
    IMAGE_SHAPE,
    STATE_NAMES,
    SkippedEpisode,
    build_action_14,
    build_joint_pose_14,
    build_reset_pose_payload,
    build_state_42,
    collect_valid_local_episode_specs,
    inspect_local_hdf5_episode,
    make_cobot_magic_left_wrist_features,
    resize_rgb_image,
)


def test_feature_schema_matches_single_camera_bimanual_hil():
    features = make_cobot_magic_left_wrist_features()

    assert list(features) == ["action", "observation.state", CAMERA_KEY]
    assert features["action"]["shape"] == (14,)
    assert features["action"]["names"] == ACTION_NAMES
    assert features["observation.state"]["shape"] == (42,)
    assert features["observation.state"]["names"] == STATE_NAMES
    assert features[CAMERA_KEY]["shape"] == IMAGE_SHAPE


def test_action_and_state_mapping_keep_full_local_bimanual_order():
    qpos = np.asarray([*range(1, 15)], dtype=np.float32)
    qvel = qpos + 100
    effort = qpos + 200

    action = build_action_14(qpos)
    np.testing.assert_allclose(action, qpos)

    state = build_state_42(qpos, qvel=qvel, effort=effort)
    assert state.shape == (42,)
    np.testing.assert_allclose(
        state[:21].reshape(7, 3),
        np.stack([qpos[:7], qvel[:7], effort[:7]], axis=1),
    )
    np.testing.assert_allclose(
        state[21:].reshape(7, 3),
        np.stack([qpos[7:], qvel[7:], effort[7:]], axis=1),
    )


def test_build_joint_pose_and_reset_payload_use_action_names():
    values = np.asarray([idx / 10 for idx in range(14)], dtype=np.float32)

    pose = build_joint_pose_14(values)
    assert list(pose) == ACTION_NAMES
    assert pose["left_joint_1.pos"] == 0.0
    assert pose["left_gripper.pos"] == np.float32(0.6)
    assert pose["right_gripper.pos"] == np.float32(1.3)

    payload = build_reset_pose_payload(values, source_episode_count=3, task="open the drawer")
    assert payload["joint_pos"] == pose
    assert payload["robot_type"] == "cobot_magic_ros_follower"
    assert payload["source"]["statistic"] == "median_first_qpos"


def test_resize_rgb_image_accepts_channel_first_float():
    image = np.zeros((3, 2, 4), dtype=np.float32)
    image[0] = 1.0

    resized = resize_rgb_image(image)

    assert resized.shape == IMAGE_SHAPE
    assert resized.dtype == np.uint8
    assert resized[..., 0].max() == 255
    assert resized[..., 1].max() == 0


def test_local_episode_inspection_filters_obvious_bad_data(tmp_path):
    h5py = __import__("h5py")
    path = tmp_path / "episode.hdf5"
    with h5py.File(path, "w") as episode:
        episode.create_dataset("action", data=np.zeros((3, 14), dtype=np.float32))
        episode.create_dataset("observations/qpos", data=np.zeros((3, 14), dtype=np.float32))
        episode.create_dataset(
            "observations/images/cam_left_wrist", data=np.zeros((3, 2, 2, 3), dtype=np.uint8)
        )

    result = inspect_local_hdf5_episode(
        path,
        min_episode_frames=4,
        max_action_delta_rad=1.0,
        max_gripper_delta=1.5,
    )

    assert isinstance(result, SkippedEpisode)
    assert "too short" in result.reason


def test_collect_valid_specs_uses_median_start_pose(tmp_path):
    h5py = __import__("h5py")
    paths = []
    for episode_idx, first_value in enumerate([0.0, 1.0, 2.0]):
        path = tmp_path / f"episode_{episode_idx}.hdf5"
        paths.append(path)
        action = np.tile(np.arange(14, dtype=np.float32), (4, 1))
        qpos = np.tile(np.arange(14, dtype=np.float32), (4, 1))
        qpos[0] = first_value
        with h5py.File(path, "w") as episode:
            episode.create_dataset("action", data=action)
            episode.create_dataset("observations/qpos", data=qpos)
            episode.create_dataset(
                "observations/images/cam_left_wrist",
                data=np.zeros((4, 2, 2, 3), dtype=np.uint8),
            )

    specs, skipped, reset_qpos = collect_valid_local_episode_specs(
        paths,
        min_episode_frames=4,
        max_action_delta_rad=1.0,
        max_gripper_delta=1.5,
        max_start_pose_deviation=None,
    )

    assert len(specs) == 3
    assert skipped == []
    np.testing.assert_allclose(reset_qpos, np.ones(14, dtype=np.float32))
