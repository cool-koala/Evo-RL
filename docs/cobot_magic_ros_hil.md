# Cobot Magic ROS Human-in-Loop

这是当前推荐的 Cobot Magic/X5 真机后端。ROS/C++ runtime 负责 CAN、关节控制、限位和重力补偿；Evo-RL 负责策略推理、动作仲裁、数据记录和人工介入。
本分支统一使用本机 ROS2 Jazzy。推荐先激活专用 conda 环境 `evo-rl-ros2-jazzy`；该环境激活脚本会自动加载 `/opt/ros/jazzy/setup.bash`。

## Runtime 位置

ROS runtime 已复制到本仓库：

```bash
third_party/cobot_magic_ros_runtime/remote_control
```

四条臂的默认映射为：`master1=右主臂/can0`、`follow1=右从臂/can1`、`master2=左主臂/can2`、`follow2=左从臂/can3`。

## 构建与启动

先进入已经配置好的 ROS2 专用环境，并检查 Python/CLI 入口：

```bash
source ~/anaconda3/etc/profile.d/conda.sh
conda activate evo-rl-ros2-jazzy
python -m ruff check \
  src/lerobot/scripts/lerobot_teleoperate.py \
  src/lerobot/scripts/lerobot_record.py \
  src/lerobot/scripts/lerobot_replay.py \
  src/lerobot/teleoperators/cobot_magic_ros \
  src/lerobot/robots/cobot_magic_ros \
  src/lerobot/utils/cobot_magic_ros.py \
  third_party/cobot_magic_ros_runtime/camera_ws/src/cobot_magic_cameras/cobot_magic_cameras/opencv_rgb_cameras.py
python -m py_compile \
  src/lerobot/scripts/lerobot_teleoperate.py \
  src/lerobot/scripts/lerobot_record.py
lerobot-teleoperate --help | rg "cobot_magic_ros|startup_sync"
lerobot-record --help | rg "cobot_magic_ros|startup_sync|reset_to_zero_pose"
```

如果环境不存在，先创建并安装依赖：`conda create -n evo-rl-ros2-jazzy python=3.12 pip`，再执行 `python -m pip install -e ".[dev,test,cobot_magic,transformers-dep]" ruff`。激活该环境后 `ROS_DISTRO` 应为 `jazzy`，`ROS_VERSION` 应为 `2`。

第一次使用或源码变更后构建：

```bash
cd third_party/cobot_magic_ros_runtime/remote_control
conda activate evo-rl-ros2-jazzy
./tools/build.sh
```

启动 CAN 和四个机械臂节点：

```bash
cd third_party/cobot_magic_ros_runtime/remote_control
conda activate evo-rl-ros2-jazzy
./tools/can.sh
./tools/remote.sh
```

如需重装 udev 规则，先运行 `./tools/set.sh`；如需核对 CANable 序列号和 `/dev/canable*` 映射，运行 `./tools/arm_serial.sh`。当前 `canable0` 使用右后主臂 serial `208138A14D4D`；若 `canable0` 缺失，先修正 `tools/arx_can.rules` 或让 `tools/can.sh` fallback 到当前 `/dev/ttyACM0`/`/dev/ttyACM1`。

启动 `remote.sh` 前必须确认 `can0` 到 `can3` 都已经存在并 `UP`：

```bash
for iface in can0 can1 can2 can3; do ip -details link show "$iface"; done
```

如果 arm 终端报 `Unable to select CAN interface can1: I/O control error` 或 `Fail to open can1`，说明 `tools/can.sh` 没有成功完成或 CAN interface 后来消失。先停止旧 arm 节点，重新执行 `./tools/arm_serial.sh`、`./tools/can.sh`，确认四个 interface 都 `UP` 后再运行 `./tools/remote.sh`。

默认 topic：

- Leader state: `/cobot_magic/leader/joint_left`, `/cobot_magic/leader/joint_right`
- Leader EE pose: `/cobot_magic/leader/end_left`, `/cobot_magic/leader/end_right`
- Leader command: `/cobot_magic/leader/command_joint_left`, `/cobot_magic/leader/command_joint_right`
- Leader manual-control: `/cobot_magic/leader/manual_control_left`, `/cobot_magic/leader/manual_control_right`
- Follower state: `/cobot_magic/puppet/joint_left`, `/cobot_magic/puppet/joint_right`
- Follower EE pose: `/cobot_magic/puppet/end_left`, `/cobot_magic/puppet/end_right`
- Follower command: `/cobot_magic/command/joint_left`, `/cobot_magic/command/joint_right`
- Cameras: `/camera_f/color/image_raw`, `/camera_l/color/image_raw`, `/camera_r/color/image_raw`

检查连接：

```bash
ros2 topic hz /cobot_magic/leader/joint_left
ros2 topic hz /cobot_magic/puppet/joint_left
ros2 topic hz /cobot_magic/leader/end_left
ros2 topic hz /cobot_magic/puppet/end_left
ros2 topic info /cobot_magic/command/joint_left
ros2 topic info /cobot_magic/leader/command_joint_left
```

## 相机启动

相机工作区现在是 ROS2 `ament_python` 包，使用 OpenCV 读取三路 RGB 设备，输出 topic 与 `cobot_magic_ros_follower` 默认相机配置一致：

```bash
cd third_party/cobot_magic_ros_runtime
./tools/build_cameras.sh
```

每次采集或 HIL 前启动相机：

```bash
cd third_party/cobot_magic_ros_runtime
./tools/camera_serial.sh
./tools/cameras.sh
./tools/check_cameras.sh
```

如果当前 shell 已经在 `third_party/cobot_magic_ros_runtime` 目录内，直接运行 `./tools/...`，不要再执行相对路径 `cd third_party/cobot_magic_ros_runtime`。

`tools/cameras.sh` 会新开一个 `cobot_magic_cameras` 终端。采集和 HIL 期间不要关闭它；关闭后重新运行 `./tools/cameras.sh`，再用 `./tools/check_cameras.sh` 确认三路 topic 都存在并持续出帧。当前相机使用 OpenCV/V4L2，图像实际频率不作为严格 30 Hz 门槛；`dataset.fps=30` 主要约束机械臂状态、action 和记录循环。默认按 `CAMERA_F_SERIAL`、`CAMERA_L_SERIAL`、`CAMERA_R_SERIAL` 在 `/dev/v4l/by-id` 和 `/dev/v4l/by-path` 中查找设备；也可以显式设置 `CAMERA_F_DEVICE=/dev/video0` 这类设备路径。

如果只发现 2 台相机，先运行 `./tools/stop_cameras.sh` 停止旧相机节点，再检查缺失相机的 USB 线、hub 供电和插口。右腕相机缺失时常见表现是 `/camera_r/color/image_raw: missing`；正常情况下 `./tools/camera_serial.sh` 应能看到三路 `/dev/video*` 或稳定的 `/dev/v4l/...` 软链接。

## 主从 Smoke Test

默认开启夹爪同步、禁用相机，只测试低速主从动作链路。`startup_sync` 会先让从臂慢速对齐主臂姿态，再进入相对跟随：

```bash
conda activate evo-rl-ros2-jazzy
lerobot-teleoperate \
  --robot.type=cobot_magic_ros_follower \
  --robot.id=cobot_magic_ros_follower_smoke \
  --robot.sync_gripper=true \
  --robot.cameras='{}' \
  --teleop.type=cobot_magic_ros_leader \
  --teleop.id=cobot_magic_ros_leader_smoke \
  --teleop.sync_gripper=true \
  --teleop.manual_control=true \
  --teleop.relative_takeover=true \
  --teleop.startup_sync=true \
  --teleop.startup_sync_duration_s=5.0 \
  --teleop.startup_sync_max_joint_delta=1.5 \
  --fps=30 \
  --teleop_time_s=15
```

如果主从姿态差超过 `startup_sync_max_joint_delta`，程序会拒绝同步；先手动摆近，或在确认安全后调大该阈值。

## 本地真机数据训练 policy

当前开抽屉 policy 不再混用网络/Piper 数据。使用本地 ARX-X5 HDF5 数据生成一个部署一致的数据集：14D action 保持本地顺序 `[left 6 joints, left gripper, right 6 joints, right gripper]`，`observation.state` 转成 ROS follower 使用的 42D 顺序，只保留左腕相机。

```bash
lerobot-convert-cobot-magic-open-drawer --overwrite
```

默认输出：

- 数据集：`data/cobot_magic_open_drawer_local_bimanual_v1/lerobot`
- repo id：`local/cobot_magic_open_drawer_local_bimanual_v1`
- 本任务 reset pose：`data/cobot_magic_open_drawer_local_bimanual_v1/reset_pose.json`

ACT 第一版建议使用较短 action chunk，先验证 HIL 稳定性：

```bash
lerobot-train \
  --dataset.repo_id=local/cobot_magic_open_drawer_local_bimanual_v1 \
  --dataset.root=data/cobot_magic_open_drawer_local_bimanual_v1/lerobot \
  --policy.type=act \
  --policy.repo_id=local/cobot_magic_open_drawer_local_bimanual_act_v1 \
  --policy.device=cuda \
  --policy.chunk_size=30 \
  --policy.n_action_steps=10 \
  --batch_size=4 \
  --steps=100000 \
  --eval_freq=0 \
  --save_freq=10000 \
  --save_checkpoint=true \
  --wandb.enable=false \
  --policy.push_to_hub=false \
  --output_dir=outputs/train/cobot_magic_open_drawer_local_bimanual_act_v1
```

Diffusion 使用同一个数据集：

```bash
lerobot-train \
  --dataset.repo_id=local/cobot_magic_open_drawer_local_bimanual_v1 \
  --dataset.root=data/cobot_magic_open_drawer_local_bimanual_v1/lerobot \
  --policy.type=diffusion \
  --policy.repo_id=local/cobot_magic_open_drawer_local_bimanual_diffusion_v1 \
  --policy.device=cuda \
  --batch_size=16 \
  --steps=100000 \
  --eval_freq=0 \
  --save_freq=10000 \
  --save_checkpoint=true \
  --wandb.enable=false \
  --policy.push_to_hub=false \
  --output_dir=outputs/train/cobot_magic_open_drawer_local_bimanual_diffusion_v1
```

上机前先离线检查 checkpoint。这个检查会在本地数据样本上比较 `pred_action`、数据 action 和当前 state；不通过时不要直接跑真机 HIL。

```bash
lerobot-check-cobot-magic-policy-sanity \
  --dataset-repo-id=local/cobot_magic_open_drawer_local_bimanual_v1 \
  --dataset-root=data/cobot_magic_open_drawer_local_bimanual_v1/lerobot \
  --policy-path=outputs/train/cobot_magic_open_drawer_local_bimanual_act_v1/checkpoints/100000/pretrained_model \
  --device=cuda \
  --output-report=outputs/train/cobot_magic_open_drawer_local_bimanual_act_v1/policy_sanity_report.json
```

## Evo-RL HIL 命令

使用 ROS 后端类型。HIL 数据录制默认读取三路相机：

- `cam_high`: `/camera_f/color/image_raw`
- `cam_left_wrist`: `/camera_l/color/image_raw`
- `cam_right_wrist`: `/camera_r/color/image_raw`

不要在 HIL 录制命令里覆盖 `--robot.cameras` 为单相机；只有纯关节调试时才加 `--robot.cameras='{}'`。当前本地 ACT/Diffusion checkpoint 的 policy 输入仍只使用 `cam_left_wrist`，但 HIL 新数据会保存三路图像，后续可用于重新训练三视角 policy。

这里的 0 位指数据采集的初始物理姿态，不要求 ROS JointState 数值全为 `0.0`。这台机器当前固定初始姿态已写入 `src/lerobot/robots/cobot_magic_ros/reset_poses/cobot_magic_ros_x5_initial_pose.json`。从臂通过 `--reset_to_zero_pose=true` 慢速插值回 JSON 里的 `joint_pos`；主臂通过 leader command topic 慢速插值回 JSON 里的 `leader_joint_pos`。

HIL 有两种主臂语义：

- `--hil_leader_mode=parked`：推荐用于 Cobot Magic。policy 驱动从臂时主臂停在初始位；按 `i` 后从臂保持当前位置，主臂慢速同步到当前从臂姿态，然后进入人工遥操作；再次按 `i` 后从臂保持当前位置，主臂慢速回初始位，然后恢复 policy。
- `--hil_leader_mode=piper`：对齐 Piper/Evo-RL 原生语义。policy 阶段通过 `--policy_sync_to_teleop=true` 让主臂持续跟随 policy/follower；按 `i` 后直接切人工接管，释放后继续跟随 policy。

```bash
lerobot-human-inloop-record \
  --robot.type=cobot_magic_ros_follower \
  --robot.id=cobot_magic_ros_follower_hil \
  --robot.sync_gripper=true \
  --robot.max_relative_target=0.02 \
  --teleop.type=cobot_magic_ros_leader \
  --teleop.id=cobot_magic_ros_leader_hil \
  --teleop.sync_gripper=true \
  --teleop.manual_control=true \
  --teleop.relative_takeover=true \
  --teleop.startup_sync=false \
  --reset_pose_path=data/cobot_magic_open_drawer_local_bimanual_v1/reset_pose.json \
  --reset_before_record=true \
  --reset_after_episode=true \
  --reset_duration_s=8.0 \
  --hil_leader_mode=parked \
  --hil_leader_sync_duration_s=5.0 \
  --hil_leader_return_duration_s=5.0 \
  --dataset.repo_id=local/eval_cobot_magic_open_drawer_hil_act_001 \
  --dataset.single_task="open the drawer" \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=10 \
  --dataset.reset_time_s=5 \
  --policy.path=outputs/train/cobot_magic_open_drawer_local_bimanual_act_v1/checkpoints/100000/pretrained_model \
  --policy.device=cuda \
  --dataset.fps=30 \
  --dataset.push_to_hub=false \
  --display_data=true
```

按 `i` 进入或退出人工接管。接管期间，主臂增量会叠加到当前从臂姿态，因此从臂不会跳到主臂的绝对姿态。`parked` 模式的主臂同步/回初始过渡阶段不写入 dataset，不会占用 episode 有效时长。

第一次上机用 `--robot.max_relative_target=0.02`。确认 policy 不再抽搐后，再逐步放宽到 `0.05`。如果需要跑 Diffusion，把 `--policy.path` 改成 Diffusion checkpoint，并先跑同样的 sanity check。

普通遥操作采集建议使用 `lerobot-record`。精简采集默认不做自动初始位 reset，开始每段前人工把主臂、从臂和任务物体放到合适起始姿态；保留 `startup_sync` 用于建立主从相对接管零点。正常数据录制默认保存三路相机；不要加单相机 `--robot.cameras` 覆盖。

```bash
lerobot-record \
  --robot.type=cobot_magic_ros_follower \
  --robot.id=cobot_magic_ros_follower_record \
  --robot.sync_gripper=true \
  --teleop.type=cobot_magic_ros_leader \
  --teleop.id=cobot_magic_ros_leader_record \
  --teleop.sync_gripper=true \
  --teleop.manual_control=true \
  --teleop.relative_takeover=true \
  --teleop.startup_sync=true \
  --teleop.startup_sync_duration_s=5.0 \
  --teleop.startup_sync_max_joint_delta=1.5 \
  --dataset.repo_id=local/cobot_magic_ros_smoke_001 \
  --dataset.single_task="cobot magic smoke test" \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=10 \
  --dataset.reset_time_s=5 \
  --dataset.fps=30 \
  --dataset.push_to_hub=false \
  --dataset.overwrite=false \
  --display_data=false
```

如果 `dataset.repo_id` 或 `dataset.root` 已存在，`lerobot-record` 会询问是否覆盖本地旧数据。输入 `y` 删除旧目录并重录，直接回车会取消。批量采集时可传 `--dataset.overwrite=true` 跳过确认。新采集数据会同时保存 joint 和 EE pose：`action` 为 28D，包含双臂 joint+gripper 目标 14D 和双臂 EE pose 14D；`observation.state` 为 56D，包含双臂 joint pos/vel/torque+gripper 42D 和双臂 EE pose 14D。EE pose 每条臂按 LeRobot EE space 保存 7 个标量：`ee.x`、`ee.y`、`ee.z`、`ee.wx`、`ee.wy`、`ee.wz`、`ee.gripper_pos`，实际字段带左右前缀，例如 `left_ee.x`、`right_ee.gripper_pos`。Cobot Magic runtime 把 `End_Effector_Pose[3:6]` 写在 `PoseStamped.orientation.x/y/z`，这里按 `wx/wy/wz` 保存，不按 ROS 四元数解释；`orientation.w` 保存夹爪位置。数据还会保存 `cam_high`、`cam_left_wrist`、`cam_right_wrist` 三路图像；早期 `sync_gripper=false` 的 12D smoke 数据只能按旧设置回放，不能和新数据直接混用。

多 episode 采集时，每段结束后不会自动复位机械臂；中间的 `dataset.reset_time_s` 是给你整理场景、摆物体和手动回到起始姿态的时间。

## 回放数据集

纯动作回放不需要相机，且要和采集时的夹爪同步设置一致；新数据统一使用 `sync_gripper=true`：

```bash
lerobot-replay \
  --robot.type=cobot_magic_ros_follower \
  --robot.id=cobot_magic_ros_follower_replay \
  --robot.sync_gripper=true \
  --robot.cameras='{}' \
  --reset_to_zero_pose=true \
  --reset_before_replay=true \
  --reset_after_replay=true \
  --reset_duration_s=8.0 \
  --require_replay_start_pose=true \
  --reset_pose_tolerance=0.05 \
  --dataset.repo_id=local/cobot_magic_ros_smoke_001 \
  --dataset.episode=0 \
  --dataset.fps=30
```

回放前会先慢速回到 JSON 固定初始姿态；`require_replay_start_pose=true` 会检查数据第一帧 action 是否接近该固定姿态，避免突然跳到第一帧动作。回放结束后会慢速回到同一初始姿态。

如果去掉 `--robot.cameras='{}'`，回放前必须确认 `/camera_f/color/image_raw`、`/camera_l/color/image_raw`、`/camera_r/color/image_raw` 都有数据。

## 注意事项

- 运行 Evo-RL/ROS 命令前使用 `conda activate evo-rl-ros2-jazzy`。该环境会自动加载 ROS2 Jazzy；runtime 脚本内部也会按 `/opt/ros/jazzy/setup.bash` 兜底加载。
- SDK 类型 `cobot_magic_follower` / `cobot_magic_leader` 仅保留诊断用途，真机 HIL 使用 `cobot_magic_ros_follower` / `cobot_magic_ros_leader`。
- `tools/can.sh` 不内置 sudo 密码；按系统 sudo 配置输入密码。
- 更完整的现场 SOP 见 [`cobot_magic_real_robot_validation.md`](cobot_magic_real_robot_validation.md)。
