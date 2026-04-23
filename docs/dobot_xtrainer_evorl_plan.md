# Dobot Xtrainer 接入 Evo-RL 人在回路方案

## 本次实现范围

- 只接入 `Dobot Xtrainer`
- `Cobot Magic` 暂不改
- 支持双臂 `joint-space`
- 支持双臂 `EE pose`
- 人在回路（Human-in-the-Loop, HIL）人工接管仍走 `joint-space`
- 自主推理前执行：
  - 从臂回初始位姿
  - 遥操作臂同步到从臂
  - 遥操作臂锁定
  - 启动策略

## 已实现接口

### Robot

- 类型：`dobot_xtrainer_follower`
- 观测：
  - `left_joint_1.pos` 到 `left_joint_6.pos`
  - `left_gripper.pos`
  - `right_joint_1.pos` 到 `right_joint_6.pos`
  - `right_gripper.pos`
  - `left_ee.x/y/z/qx/qy/qz/qw`
  - `right_ee.x/y/z/qx/qy/qz/qw`
  - `cam_high`
  - `cam_left_wrist`
  - `cam_right_wrist`
- 动作模式：
  - `action_mode=joint`
  - `action_mode=ee_pose`
- 安全：
  - 关节步长限幅
  - `EE pose` 平移/姿态步长限幅
  - 关节安全位检查
  - 指示灯状态控制

### Teleoperator

- 类型：`dobot_xtrainer_leader`
- 输入：
  - 双臂 14 维 joint-space
- 反馈：
  - 从臂实际 joint 状态镜像到 leader
- 控制事件：
  - `Button A` 短按：切换人工介入
  - `Button A` 长按：同步 leader 到 follower
  - `Button B` 短按：结束当前录制段

## 保护

- 不做真力控闭环
- 采用软件保护与外接传感器保护：
  - 同步时渐进逼近
  - 工作空间保护
  - TCP 速度保护
  - 单步关节增量保护
  - 单步笛卡尔位移保护
  - 卡阻检测
  - `using_sensor` 触发立即脱力
- 保护触发后：
  - leader 立即停止镜像
  - leader 脱力
  - 指示灯转红
  - 当前 episode 退出
  - 需要人工清障后长按 `Button A` 同步恢复

## Evo-RL 核心最小修改

- `Teleoperator` 增加可选接口：
  - `poll_control_events`
  - `sync_to_robot`
  - `prepare_for_autonomous_start`
- `Robot` 增加可选接口：
  - `normalize_action_for_storage`
  - `get_feedback_action_for_teleop`
  - `set_indicator_state`
- `recording_loop.py`
  - 轮询 teleoperator 按钮事件
  - 在策略/人工接管状态间更新指示灯
  - 允许 teleop 触发同步
  - 在 `ee_pose` 模式下把人工接管动作归一化到数据集动作空间
- `recording_hil.py`
  - policy 同步 leader 时优先使用 follower 实际 joint 反馈
- `lerobot_record.py`
  - 每个策略 episode 开始前执行自主推理启动序列

## 说明

- 为了保证本地可运行性，这次接入做成了 Evo-RL 仓内的本地设备模块，而不是单独的 editable 第三方包。
- 对外暴露的设备类型与接口保持独立，后续如果需要再拆成独立插件，迁移成本很低。
