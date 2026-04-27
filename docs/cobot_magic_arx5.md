# Cobot Magic ARX-5 使用指南

本文说明如何在 EvoRL/LeRobot 中使用 Cobot Magic 双臂机械臂。当前实现面向两条 ARX-5/X5 机械臂：一组作为 `cobot_magic_leader`，一组作为 `cobot_magic_follower`。

## 1. 安装依赖

先安装本项目和 Cobot Magic 可选依赖。该 extra 会在 Linux 上安装 ARX5 官方 PyPI wheel；非 Linux 环境会跳过 SDK wheel，仍可运行无硬件单测：

```bash
pip install -e ".[dev,test,cobot_magic]"
```

如果你的环境不能从 PyPI 安装 `arx5-interface`，请按 ARX5 SDK 官方仓库构建，并确保 Python 能 import：

```bash
python -c "import arx5_interface; print('ARX5 SDK ok')"
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

不要把左右臂配置到同一个 interface，否则程序会拒绝启动。

## 3. Teleoperate 使用

下面示例假设 follower 使用 `can0/can1`，leader 使用 `can2/can3`：

```bash
lerobot-teleoperate \
  --robot.type=cobot_magic_follower \
  --robot.id=cobot_magic_follower \
  --robot.left_arm_config.interface=can0 \
  --robot.right_arm_config.interface=can1 \
  --teleop.type=cobot_magic_leader \
  --teleop.id=cobot_magic_leader \
  --teleop.left_arm_config.interface=can2 \
  --teleop.right_arm_config.interface=can3 \
  --fps=30
```

Leader 默认进入 `manual_control=true`，也就是 ARX SDK 的 damping 示教模式；`gravity_compensation=true` 默认开启，由 ARX SDK 执行重力补偿。Follower 接收 leader 读出的关节位置并执行。

## 4. 录制数据

相机继续用 EvoRL 标准 `CameraConfig`，不走旧的 Cobot Magic ROS 图像 topic。示例：

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
pytest -q tests/test_cobot_magic.py tests/utils/test_control_utils.py
```

真实硬件 smoke test 建议先降低频率并空载执行：

```bash
lerobot-teleoperate \
  --robot.type=cobot_magic_follower \
  --robot.left_arm_config.interface=can0 \
  --robot.right_arm_config.interface=can1 \
  --teleop.type=cobot_magic_leader \
  --teleop.left_arm_config.interface=can2 \
  --teleop.right_arm_config.interface=can3 \
  --fps=10 \
  --teleop_time_s=10
```

确认两侧关节方向、夹爪开合方向和急停可用后，再提高 `fps` 或开始录制。

## 7. 数据字段

动作是 14 个位置目标：`left_joint_1.pos` 到 `left_joint_6.pos`、`left_gripper.pos`，以及右臂同名 `right_...` 字段。观测除位置外，还会记录每个关节/夹爪的 `vel` 和 `torque`，方便检查真实硬件状态。

## 8. 安全注意事项

- 默认型号是 `X5`。如果你的硬件不是 X5，必须显式修改 `left_arm_config.model` 和 `right_arm_config.model`，选错型号可能导致危险动作。
- `max_relative_target` 默认限制单帧目标跳变；如果策略输出抖动，不要直接调大，先检查数据和 policy。
- `reset_to_home_on_connect=false` 是刻意的安全默认值，避免连接时机械臂突然回 home。
- 断开时默认调用 `set_to_damping()`，让机械臂进入较安全的阻尼状态。
- EvoRL 不自动执行 ARX SDK 的关节校准。需要校准时，请先使用 ARX 官方工具确认流程。
