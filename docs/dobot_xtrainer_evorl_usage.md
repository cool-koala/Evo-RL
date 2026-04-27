# Dobot Xtrainer Evo-RL 使用说明

## 1. 环境准备

默认假设目录结构如下：

```text
/home/abc/guoxiaoyu/Dobot_Xtrainer/
├── Evo-RL/
└── dobot_xtrainer-master/
```

如果 `dobot_xtrainer-master` 不在这个位置，启动时显式传入 `--robot.dobot_root` 和 `--teleop.dobot_root`。

运行前建议设置：

```bash
export PYTHONPATH=/home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/src:/home/abc/guoxiaoyu/Dobot_Xtrainer/dobot_xtrainer-master
```

## 2. 新增类型

- follower：`dobot_xtrainer_follower`
- leader：`dobot_xtrainer_leader`

## 3. 关键配置

### follower

- `--robot.type=dobot_xtrainer_follower`
- `--robot.action_mode=joint|ee_pose`
- `--robot.use_cameras=true|false`
- `--robot.move_to_home_on_connect=true|false`
- `--robot.robot_command_hz=<hz>`
- `--robot.camera_fps=<hz>`

### leader

- `--teleop.type=dobot_xtrainer_leader`
- `--teleop.manual_control=true|false`
- `--teleop.teleop_poll_hz=<hz>`
- `--teleop.leader_sensor_enabled=true|false`
- `--teleop.leader_sync_max_joint_delta_rad=<rad>`
- `--teleop.leader_sync_max_rot_step_deg=<deg>`

## 4. 按键与灯色

### 按键

- `Button A` 短按：开始/结束人工介入
- `Button A` 长按：同步遥操作臂到当前从臂
- `Button B` 短按：结束当前 episode/录制段

### 灯色

- 黄色：自主推理/策略控制
- 绿色：人工介入或纯手动阶段
- 红色：保护触发、卡阻或外接传感器触发

## 5. 启动流程

### 纯 joint-space 遥操作/录制

```bash
python -m lerobot.scripts.lerobot_record \
  --robot.type=dobot_xtrainer_follower \
  --robot.action_mode=joint \
  --teleop.type=dobot_xtrainer_leader \
  --dataset.repo_id=<user>/<dataset> \
  --dataset.single_task="teleop task" \
  --dataset.num_episodes=1
```

### 策略推理 + HIL

```bash
python -m lerobot.scripts.lerobot_record \
  --robot.type=dobot_xtrainer_follower \
  --robot.action_mode=ee_pose \
  --teleop.type=dobot_xtrainer_leader \
  --policy.path=<policy_path> \
  --policy_sync_to_teleop=true \
  --dataset.repo_id=<user>/<dataset> \
  --dataset.single_task="policy task" \
  --dataset.num_episodes=1
```

## 6. 自主推理时的实际行为

当 `policy + teleop` 同时启用时，每个 episode 开始前都会执行：

1. 从臂回初始位姿
2. leader 同步到 follower 当前 joint 状态
3. leader 锁定
4. 指示灯切黄
5. 启动策略

如果 `action_mode=ee_pose`：

- 策略向 follower 发送 `EE pose`
- leader 不直接跟随 `EE pose`
- leader 跟随 follower 当前实际 joint 状态
- 人工介入时仍然输出 `joint-space`
- `EE pose` 四元数会在发送前归一化，非法数值会被拒绝

## 7. 观测与保护

- follower 观测同时包含 joint、EE pose 和三路相机图像。
- 夹爪观测会直接读取真实夹爪硬件位置，并按动作接口方向归一化到 `0~1`。
- 如果夹爪读取失败，adapter 会回退到最后一次已发送动作或当前 joint fallback，不会继续使用底层固定 `1.0`。
- `camera_fps` 控制相机后台线程读取频率；主循环仍按 dataset/control fps 运行。
- Button A 长按同步失败、leader 卡阻、传感器触发或安全检查失败都会红灯、脱力并退出当前 episode。

## 8. 保护触发后的恢复

1. 清除 leader 周围障碍物
2. 确认外接传感器恢复正常
3. 长按 `Button A` 重新同步
4. 再开始下一段录制或策略 episode

## 9. 真机联调顺序

先不要直接上策略模型，按下面顺序逐步打开能力。

### 9.1 环境和硬件检查

```bash
conda activate dobot_rl
cd /home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL
export PYTHONPATH=/home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/src:/home/abc/guoxiaoyu/Dobot_Xtrainer/dobot_xtrainer-master

ping -c 2 192.168.5.1
ping -c 2 192.168.5.2
ls /dev/ttyACM* /dev/ttyUSB*
python -c "import pyrealsense2 as rs; print([d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()])"
```

### 9.2 第一轮低风险 joint 测试

不开相机、不自动回 home，先确认 follower/leader 连接、Button A、Button B 和保护行为。

```bash
python -m lerobot.scripts.lerobot_record \
  --robot.type=dobot_xtrainer_follower \
  --robot.dobot_root=/home/abc/guoxiaoyu/Dobot_Xtrainer/dobot_xtrainer-master \
  --robot.action_mode=joint \
  --robot.use_cameras=false \
  --robot.move_to_home_on_connect=false \
  --teleop.type=dobot_xtrainer_leader \
  --teleop.dobot_root=/home/abc/guoxiaoyu/Dobot_Xtrainer/dobot_xtrainer-master \
  --dataset.repo_id=local/dobot_joint_smoke \
  --dataset.root=/tmp/dobot_joint_smoke \
  --dataset.single_task="joint smoke test" \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=0 \
  --dataset.push_to_hub=false
```

### 9.3 第二轮打开相机

确认三路图像稳定后再录制数据。相机频率可以先降到 `15`。

```bash
python -m lerobot.scripts.lerobot_record \
  --robot.type=dobot_xtrainer_follower \
  --robot.dobot_root=/home/abc/guoxiaoyu/Dobot_Xtrainer/dobot_xtrainer-master \
  --robot.action_mode=joint \
  --robot.use_cameras=true \
  --robot.camera_fps=15 \
  --robot.move_to_home_on_connect=false \
  --teleop.type=dobot_xtrainer_leader \
  --teleop.dobot_root=/home/abc/guoxiaoyu/Dobot_Xtrainer/dobot_xtrainer-master \
  --dataset.repo_id=local/dobot_camera_smoke \
  --dataset.root=/tmp/dobot_camera_smoke \
  --dataset.single_task="camera smoke test" \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=0 \
  --dataset.push_to_hub=false
```

### 9.4 第三轮测试自主启动序列

确认空间安全后再打开 `move_to_home_on_connect=true`，并用 Button A 长按测试同步。策略测试先用“保持当前位置”的 dummy policy 或云端 adapter，不要用随机模型直接闭环控制真机。

## 10. 8GB 4090 Laptop 模型建议

- 第一轮真机策略测试：使用保持当前位置的 dummy policy/adapter。
- 本地可优先尝试：`ACT`，显存和延迟最适合作为 8GB 笔记本真机冒烟模型。
- 可谨慎尝试：`diffusion`，建议把 inference steps 降到 `4~8`，控制频率先用 `10Hz`。
- 不建议本地首轮闭环：`SmolVLA`、`pi0`、`pi0_fast`、`EVO1`、`GROOT`、`X-VLA`。这些更适合云端或更大显存机器，边端只负责机械臂、相机、按钮和保护。

## 11. 已知限制

- `EE pose` 首版只作为策略/自主推理接口，不作为真人输入空间
- `Button B` 当前实现为“结束当前 episode/录制段”，不是暂停/恢复录制开关
- leader 保护首版为软件保护 + 外接传感器保护，不是真正的实时力控闭环
