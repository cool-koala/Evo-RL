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

### leader

- `--teleop.type=dobot_xtrainer_leader`
- `--teleop.manual_control=true|false`
- `--teleop.teleop_poll_hz=<hz>`
- `--teleop.leader_sensor_enabled=true|false`
- `--teleop.leader_sync_max_joint_delta_rad=<rad>`

## 4. 按键与灯色

### 按键

- `Button A` 短按：开始/结束人工介入
- `Button A` 长按：同步遥操作臂到当前从臂
- `Button B` 短按：结束当前录制段

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

## 7. 保护触发后的恢复

1. 清除 leader 周围障碍物
2. 确认外接传感器恢复正常
3. 长按 `Button A` 重新同步
4. 再开始下一段录制或策略 episode

## 8. 已知限制

- `EE pose` 首版只作为策略/自主推理接口，不作为真人输入空间
- `Button B` 当前实现为“结束当前录制段”，不直接等价于键盘 `ESC` 的整场停止
- leader 保护首版为软件保护 + 外接传感器保护，不是真正的实时力控闭环
