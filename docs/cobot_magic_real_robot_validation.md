# Cobot Magic 真机 RL 验证教程

本文用于现场验证 Cobot Magic 在 EvoRL 中是否可以完成真机主从跟随、数据采集、回放和 human-in-loop RL rollout。本文统一称 `cobot_magic_leader` 为主臂，`cobot_magic_follower` 为从臂：人拖动主臂，从臂执行跟随或策略动作。

## 0. 验证前检查

先确认现场具备急停、空载机械臂、足够运动空间，并且操作员手能随时触发断电或急停。首次验证不要放任务物体，不要让机械臂靠近人或硬物。

```bash
git branch --show-current
python3 -c "import arx5_interface; print('ARX5 SDK ok')"
python3 -m pytest -q tests/test_cobot_magic.py tests/utils/test_control_utils.py
```

如果 `pytest` 不存在，先安装开发依赖：

```bash
python3 -m pip install -e ".[dev,test,cobot_magic]"
```

通过标准：SDK 能 import，mock 单测通过。单测只验证 EvoRL 适配层逻辑，不代表真机安全。

## 1. CAN 接口确认

记录四条机械臂的接口名，避免左右臂或主从臂接反：

```bash
ip -details link show
export CM_FOLLOWER_LEFT=can0
export CM_FOLLOWER_RIGHT=can1
export CM_LEADER_LEFT=can2
export CM_LEADER_RIGHT=can3
```

如需手动拉起 SocketCAN，bitrate 以现场硬件配置为准：

```bash
sudo ip link set "$CM_FOLLOWER_LEFT" up type can bitrate 1000000
sudo ip link set "$CM_FOLLOWER_RIGHT" up type can bitrate 1000000
sudo ip link set "$CM_LEADER_LEFT" up type can bitrate 1000000
sudo ip link set "$CM_LEADER_RIGHT" up type can bitrate 1000000
```

通过标准：四个 interface 都是 `UP`，左右从臂、左右主臂以及主从之间都不能共用同一个 interface；程序会在启动前拒绝重复配置。

## 2. 低速主从跟随 Smoke Test

先让四条臂摆到接近的安全姿态，夹爪先不同步，只验证 6 个 arm joints。`max_relative_target=0.05` 会把单帧目标跳变限制在较小范围内。

```bash
lerobot-teleoperate \
  --robot.type=cobot_magic_follower \
  --robot.id=cobot_magic_follower_smoke \
  --robot.left_arm_config.interface="$CM_FOLLOWER_LEFT" \
  --robot.right_arm_config.interface="$CM_FOLLOWER_RIGHT" \
  --robot.left_arm_config.sync_gripper=false \
  --robot.right_arm_config.sync_gripper=false \
  --robot.left_arm_config.max_relative_target=0.05 \
  --robot.right_arm_config.max_relative_target=0.05 \
  --teleop.type=cobot_magic_leader \
  --teleop.id=cobot_magic_leader_smoke \
  --teleop.left_arm_config.interface="$CM_LEADER_LEFT" \
  --teleop.right_arm_config.interface="$CM_LEADER_RIGHT" \
  --teleop.left_arm_config.sync_gripper=false \
  --teleop.right_arm_config.sync_gripper=false \
  --fps=10 \
  --teleop_time_s=15
```

通过标准：

- 主臂进入可拖动状态，没有明显下坠、自激或突然回零。
- 从臂左右方向正确，6 个关节跟随方向正确。
- 从臂动作平滑，没有大跳变、持续抖动或通信报错。
- 按 `Ctrl+C` 退出后机械臂进入阻尼/安全状态。

若任一关节方向相反、左右臂接反、夹爪异常或出现快速跳变，立即停止，不进入下一步。

## 3. 全量主从跟随与数据采集

确认 arm joints 正常后，再打开夹爪同步并录制短数据集：

```bash
lerobot-record \
  --robot.type=cobot_magic_follower \
  --robot.id=cobot_magic_follower_record \
  --robot.left_arm_config.interface="$CM_FOLLOWER_LEFT" \
  --robot.right_arm_config.interface="$CM_FOLLOWER_RIGHT" \
  --robot.left_arm_config.max_relative_target=0.08 \
  --robot.right_arm_config.max_relative_target=0.08 \
  --robot.cameras='{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: "MJPG"}}' \
  --teleop.type=cobot_magic_leader \
  --teleop.id=cobot_magic_leader_record \
  --teleop.left_arm_config.interface="$CM_LEADER_LEFT" \
  --teleop.right_arm_config.interface="$CM_LEADER_RIGHT" \
  --dataset.repo_id=local/cobot_magic_smoke \
  --dataset.single_task="cobot magic smoke test" \
  --dataset.num_episodes=2 \
  --dataset.episode_time_s=10 \
  --dataset.reset_time_s=5 \
  --dataset.push_to_hub=false \
  --display_data=true
```

采集后检查数据：

```bash
lerobot-dataset-report --dataset local/cobot_magic_smoke
lerobot-dataset-viz --repo-id local/cobot_magic_smoke --episode-index 0 --display-compressed-images 0
```

通过标准：数据集中有 14 维 action，也就是左右各 6 个关节加 1 个夹爪；图像能正常显示；episode 时长、fps、action 曲线没有明显断帧或阶跃。

## 4. 回放验证

回放第 0 个 episode，确认从臂能复现刚才的动作。第一次回放仍保持低速空间和急停准备。

```bash
lerobot-replay \
  --robot.type=cobot_magic_follower \
  --robot.id=cobot_magic_follower_replay \
  --robot.left_arm_config.interface="$CM_FOLLOWER_LEFT" \
  --robot.right_arm_config.interface="$CM_FOLLOWER_RIGHT" \
  --robot.left_arm_config.max_relative_target=0.08 \
  --robot.right_arm_config.max_relative_target=0.08 \
  --dataset.repo_id=local/cobot_magic_smoke \
  --dataset.episode=0 \
  --dataset.fps=10
```

通过标准：回放动作方向、幅度和夹爪行为与采集时一致；退出后机械臂没有保持危险高刚度状态。

## 5. 真机 RL / Human-in-loop Rollout

有 policy checkpoint 后，用 human-in-loop 模式验证。该模式会执行 policy，同时把 policy action 同步给主臂；按 `i` 可切换人工接管，按 `s` 标记成功并结束 episode，按 `f` 标记失败并结束 episode，按 `Esc` 停止。

```bash
lerobot-human-inloop-record \
  --robot.type=cobot_magic_follower \
  --robot.id=cobot_magic_follower_rl \
  --robot.left_arm_config.interface="$CM_FOLLOWER_LEFT" \
  --robot.right_arm_config.interface="$CM_FOLLOWER_RIGHT" \
  --robot.left_arm_config.max_relative_target=0.05 \
  --robot.right_arm_config.max_relative_target=0.05 \
  --robot.cameras='{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: "MJPG"}}' \
  --teleop.type=cobot_magic_leader \
  --teleop.id=cobot_magic_leader_rl \
  --teleop.left_arm_config.interface="$CM_LEADER_LEFT" \
  --teleop.right_arm_config.interface="$CM_LEADER_RIGHT" \
  --teleop.left_arm_config.max_relative_target=0.05 \
  --teleop.right_arm_config.max_relative_target=0.05 \
  --policy.path=/path/to/policy/pretrained_model \
  --dataset.repo_id=local/cobot_magic_rl_smoke \
  --dataset.single_task="cobot magic rl smoke test" \
  --dataset.num_episodes=3 \
  --dataset.episode_time_s=15 \
  --dataset.reset_time_s=8 \
  --dataset.push_to_hub=false \
  --display_data=true
```

首次运行时程序会要求把所有机械臂放到 reset pose 并按回车保存。之后每个 episode 被标记成功或失败后，会慢速回到该 reset pose。

通过标准：

- policy 能加载并输出动作，程序没有 feature mismatch。
- 从臂执行 policy 时动作连续，主臂能同步 policy 动作。
- 按 `i` 后人工主臂能接管从臂；再次按 `i` 后能回到 policy。
- 数据集包含 `complementary_info.policy_action`、`complementary_info.is_intervention`、`complementary_info.state` 和 episode success/failure 标签。

## 6. 失败处理与边界

- 当前 EvoRL 适配层主要提供位置目标限幅和主从/策略链路验证，不提供碰撞力控或人体接触力控认证。
- 发现方向错误、通信掉线、夹爪单位不对、动作抖动或电流异常时，立即停机，先回到第 2 步低速验证。
- 在 10 fps、15 秒短 episode 稳定前，不要提高 fps、放置硬物任务场景或执行长时间 RL rollout。
- 真机 RL 通过后，再逐步把 `fps` 提到任务需要的频率，并把 `max_relative_target` 从 `0.05` 调到经过现场验证的值。
