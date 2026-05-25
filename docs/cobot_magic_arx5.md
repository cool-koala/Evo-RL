# Cobot Magic ARX5 SDK 诊断指南

本文保留 ARX5 Python SDK 后端的安装、接口和单测说明，仅用于适配层诊断。现场 Cobot Magic/X5 真机主从、数据采集、回放和 HIL rollout 不再走本路径，推荐使用 ROS/C++ runtime：[`cobot_magic_ros_hil.md`](cobot_magic_ros_hil.md)。

历史类型 `cobot_magic_leader` / `cobot_magic_follower` 仍可用于 SDK import、mock 单测和低风险诊断，但现场测试发现 SDK 内部重力补偿不能满足当前主从跟随目标。真机验证请按 [Cobot Magic 真机 RL 验证教程](cobot_magic_real_robot_validation.md) 执行。

## 1. 安装依赖

先安装本项目和 Cobot Magic 可选依赖。该 extra 会在 Linux 上安装 ARX5 官方 PyPI wheel；非 Linux 环境会跳过 SDK wheel，仍可运行无硬件单测：

```bash
pip install -e ".[dev,test,cobot_magic]"
```

如果你的环境不能从 PyPI 安装 `arx5-interface`，请按 ARX5 SDK 官方仓库构建，并确保 Python 能 import：

```bash
python3 -c "import arx5_interface; print('ARX5 SDK ok')"
```

ARX5 SDK 参考：https://github.com/real-stanford/arx5-sdk

## 2. 准备 CAN 接口

ARX-5 SDK 直接使用 CAN 或 EtherCAT-CAN 接口。先确认每条机械臂的接口名：

```bash
ip a
```

常见 USB-CAN 启动方式如下，接口和 bitrate 以你的硬件为准：

```bash
sudo ip link set up can0 type can bitrate 1000000
sudo ip link set up can1 type can bitrate 1000000
sudo ip link set up can2 type can bitrate 1000000
sudo ip link set up can3 type can bitrate 1000000
```

不要让 follower 左右臂、leader 左右臂或主从之间共用同一个 interface，否则程序会拒绝启动。

## 3. Teleoperate 诊断

下面示例只用于 SDK 路径诊断，不作为当前真机 HIL SOP。示例假设 follower 使用 `can0/can1`，leader 使用 `can2/can3`：

```bash
export CM_ARM_MODEL=X5

lerobot-teleoperate \
  --robot.type=cobot_magic_follower \
  --robot.id=cobot_magic_follower \
  --robot.left_arm_config.model="$CM_ARM_MODEL" \
  --robot.right_arm_config.model="$CM_ARM_MODEL" \
  --robot.left_arm_config.interface=can0 \
  --robot.right_arm_config.interface=can1 \
  --robot.left_arm_config.relative_target_mode=true \
  --robot.right_arm_config.relative_target_mode=true \
  --robot.left_arm_config.max_relative_target=0.10 \
  --robot.right_arm_config.max_relative_target=0.10 \
  --teleop.type=cobot_magic_leader \
  --teleop.id=cobot_magic_leader \
  --teleop.left_arm_config.model="$CM_ARM_MODEL" \
  --teleop.right_arm_config.model="$CM_ARM_MODEL" \
  --teleop.left_arm_config.interface=can2 \
  --teleop.right_arm_config.interface=can3 \
  --fps=25
```

Leader 默认进入 `manual_control=true`，也就是 ARX SDK 的 damping 示教模式；`gravity_compensation=true` 默认开启，由 ARX SDK 执行重力补偿。Follower 在 `relative_target_mode=true` 时执行 leader 的关节增量，适合主从初始姿态不完全一致的现场示教。

## 4. 录制数据诊断

相机继续用 EvoRL 标准 `CameraConfig`。此命令只用于验证 SDK 后端能否采集，不用于当前主线 HIL：

```bash
lerobot-record \
  --robot.type=cobot_magic_follower \
  --robot.id=cobot_magic_follower \
  --robot.left_arm_config.interface=can0 \
  --robot.right_arm_config.interface=can1 \
  --robot.cameras='{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
  --teleop.type=cobot_magic_leader \
  --teleop.id=cobot_magic_leader \
  --teleop.left_arm_config.interface=can2 \
  --teleop.right_arm_config.interface=can3 \
  --dataset.repo_id=${HF_USER}/cobot-magic-test \
  --dataset.single_task="pick up the object" \
  --dataset.num_episodes=5 \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=5 \
  --dataset.push_to_hub=false
```

## 5. 回放数据

```bash
lerobot-replay \
  --robot.type=cobot_magic_follower \
  --robot.id=cobot_magic_follower \
  --robot.left_arm_config.interface=can0 \
  --robot.right_arm_config.interface=can1 \
  --dataset.repo_id=${HF_USER}/cobot-magic-test \
  --dataset.episode=0 \
  --dataset.fps=30
```

## 6. 测试

无硬件单元测试使用 mock ARX SDK：

```bash
python3 -m pytest -q tests/test_cobot_magic.py tests/utils/test_control_utils.py
```

真实硬件 smoke test 仅限诊断。主从跟随、重力补偿和 HIL 请使用 ROS 后端：

```bash
lerobot-teleoperate \
  --robot.type=cobot_magic_follower \
  --robot.left_arm_config.model="$CM_ARM_MODEL" \
  --robot.right_arm_config.model="$CM_ARM_MODEL" \
  --robot.left_arm_config.interface=can0 \
  --robot.right_arm_config.interface=can1 \
  --robot.left_arm_config.relative_target_mode=true \
  --robot.right_arm_config.relative_target_mode=true \
  --teleop.type=cobot_magic_leader \
  --teleop.left_arm_config.model="$CM_ARM_MODEL" \
  --teleop.right_arm_config.model="$CM_ARM_MODEL" \
  --teleop.left_arm_config.interface=can2 \
  --teleop.right_arm_config.interface=can3 \
  --fps=25 \
  --teleop_time_s=10
```

如果该 SDK 路径出现重力补偿不足、只能部分关节跟随或频率过低，不要继续在本路径调参，切回 ROS 后端排查。

## 7. 数据字段

动作是 14 个位置目标：`left_joint_1.pos` 到 `left_joint_6.pos`、`left_gripper.pos`，以及右臂同名 `right_...` 字段。观测除位置外，还会记录每个关节/夹爪的 `vel` 和 `torque`，方便检查真实硬件状态。

## 8. 安全注意事项

- Cobot Magic 现场机械臂使用 `X5`。不要切到 `L5`，两者前 3 个关节电机类型不同，选错型号可能导致危险动作。
- 如果机械臂底座不是 SDK 默认竖直安装方向，使用 `gravity_vector` 覆盖重力方向，例如 `--teleop.left_arm_config.gravity_vector='[0, 9.807, 0]'`。
- `max_relative_target` 默认限制单帧目标跳变；如果策略输出抖动，不要直接调大，先检查数据和 policy。
- `reset_to_home_on_connect=false` 是刻意的安全默认值，避免连接时机械臂突然回 home。
- 断开时默认调用 `set_to_damping()`，让机械臂进入较安全的阻尼状态。
- EvoRL 不自动执行 ARX SDK 的关节校准。需要校准时，请先使用 ARX 官方工具确认流程。
