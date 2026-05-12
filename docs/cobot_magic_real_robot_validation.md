# Cobot Magic 真机 RL 验证教程

本文是当前推荐的 Cobot Magic/X5 真机验证 SOP。真机主从、采集、回放和 human-in-loop rollout 使用 ROS/C++ runtime 提供 CAN、关节控制、限位和重力补偿，Evo-RL 只负责策略推理、动作仲裁和人工介入。ARX5 Python SDK 路径仅保留为诊断入口，见 [`cobot_magic_arx5.md`](cobot_magic_arx5.md)。

## 0. 安全与环境检查

首次验证保持空载、低速、无遮挡，并确保操作员可以立即扶住机械臂或触发急停。不要在跟随稳定前放置任务物体。

```bash
conda activate evo-rl
python -m pip install -e ".[dev,test,cobot_magic]"
python -m pip install ruff
source /opt/ros/noetic/setup.zsh
python -m pytest -q tests/test_cobot_magic_ros.py tests/utils/test_control_utils.py
python -m ruff check \
  src/lerobot/scripts/robot_reset.py \
  src/lerobot/scripts/lerobot_record.py \
  src/lerobot/scripts/lerobot_replay.py \
  src/lerobot/scripts/lerobot_human_inloop_record.py \
  tests/test_cobot_magic_ros.py
```

如果当前终端是 bash，再改用 `source /opt/ros/noetic/setup.bash`。不要在 zsh 里 source `setup.bash`。

通过标准：ROS 后端单测和 Ruff 检查通过；这只验证 Evo-RL 适配层，不代表真机运动安全。如果 `python -m ruff` 提示缺模块，先在当前 `evo-rl` 环境中执行上面的 `python -m pip install ruff`。

## 1. 硬件映射

现场四条 X5 机械臂按以下顺序连接，后排是主臂，前排是从臂：

| CAN | 位置 | 角色 | ROS workspace | 默认 topic |
| --- | --- | --- | --- | --- |
| `can0` | 右后 | 右主臂 | `master1` | `/cobot_magic/leader/joint_right` |
| `can1` | 右前 | 右从臂 | `follow1` | `/cobot_magic/puppet/joint_right` |
| `can2` | 左后 | 左主臂 | `master2` | `/cobot_magic/leader/joint_left` |
| `can3` | 左前 | 左从臂 | `follow2` | `/cobot_magic/puppet/joint_left` |

`tools/can.sh` 默认使用 `/dev/canable0` 到 `/dev/canable3` 拉起上述 CAN 口。当前右后主臂 `canable0` 的 serial 为 `208138A14D4D`；如果现场更换 CANable 或 udev 映射变化，先运行：

```bash
cd third_party/cobot_magic_ros_runtime/remote_control
./tools/arm_serial.sh
```

核对 `/dev/canable*` 指向。如 `canable0` 缺失但其余 `canable1-3` 正常，通常是右后主臂 serial 与 `tools/arx_can.rules` 不一致；修正规则后执行 `./tools/set.sh`，再重新插拔 CANable 或触发 udev。

## 2. 构建并启动 ROS Runtime

前置条件：四个 CANable 都已插好，`/dev/canable1`、`/dev/canable2`、`/dev/canable3` 必须存在；`/dev/canable0` 存在最好，如果缺失，`tools/can.sh` 会临时 fallback 到 `/dev/ttyACM0` 或 `/dev/ttyACM1`。每次启动 arm ROS 节点前必须先成功执行 `tools/can.sh`，并确认 `can0` 到 `can3` 都是 `UP`。

```bash
cd third_party/cobot_magic_ros_runtime/remote_control
source /opt/ros/noetic/setup.zsh
./tools/build.sh
./tools/can.sh
for iface in can0 can1 can2 can3; do ip -details link show "$iface"; done
./tools/remote.sh
```

`tools/remote.sh` 会启动两条主臂和两条从臂节点，从臂使用 `control_mode:=2` 接收 Evo-RL 发布的关节目标。

如果四个 arm 终端出现 `Unable to select CAN interface can1: I/O control error` 或 `Fail to open can1`，说明 arm node 启动时 SocketCAN interface 不存在或未 `UP`。处理顺序：

```bash
cd third_party/cobot_magic_ros_runtime/remote_control
./tools/arm_serial.sh
pgrep -af 'slcand|roslaunch arm_control|arm_node' || true
for iface in can0 can1 can2 can3; do ip -details link show "$iface" || true; done
./tools/can.sh
for iface in can0 can1 can2 can3; do ip -details link show "$iface"; done
./tools/remote.sh
```

不要在 `tools/can.sh` 失败后继续运行 `tools/remote.sh`；这样只会让四个 ROS arm node 反复报 `Fail to open can*`。

检查 topic：

```bash
rostopic hz /cobot_magic/leader/joint_right
rostopic hz /cobot_magic/leader/joint_left
rostopic hz /cobot_magic/puppet/joint_right
rostopic hz /cobot_magic/puppet/joint_left
rostopic info /cobot_magic/command/joint_right
rostopic info /cobot_magic/command/joint_left
```

通过标准：四个 state topic 都有稳定频率，两个 command topic 有 subscriber。

## 3. 启动相机 ROS 节点

当前这台机器使用 3 个奥比中光 Astra/Orbbec 相机，默认映射为：

| 视角 | 序列号 | ROS topic | LeRobot key |
| --- | --- | --- | --- |
| 前视 | `AU1SB3300XB` | `/camera_f/color/image_raw` | `cam_high` |
| 左腕 | `AU1SB3300YB` | `/camera_l/color/image_raw` | `cam_left_wrist` |
| 右腕 | `AU1SB33005A` | `/camera_r/color/image_raw` | `cam_right_wrist` |

第一次使用或相机源码变更后构建一次：

```bash
cd third_party/cobot_magic_ros_runtime
./tools/build_cameras.sh
```

每次真机采集前启动相机：

```bash
cd /home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/third_party/cobot_magic_ros_runtime
./tools/camera_serial.sh
./tools/cameras.sh
```

如果当前 shell 已经在 `third_party/cobot_magic_ros_runtime` 目录内，直接运行 `./tools/camera_serial.sh` 和 `./tools/cameras.sh`，不要再执行相对路径 `cd third_party/cobot_magic_ros_runtime`。

`tools/cameras.sh` 会打开一个新的 `cobot_magic_cameras_astra` 终端运行相机 ROS 节点。采集期间不要关闭这个终端；如果已经关掉，重新运行 `./tools/cameras.sh`。启动前脚本会检查三台相机 serial，任一缺失都会退出，不会启动半残的相机栈。

如果 `camera_serial.sh` 只显示 2 台设备，或 `check_cameras.sh` 显示 `/camera_r/color/image_raw: missing`，说明系统没有枚举到右腕相机 `AU1SB33005A`。先停止旧相机节点，再检查右腕相机 USB 线、USB hub 供电和插口；重新插拔后再次检查：

```bash
cd /home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/third_party/cobot_magic_ros_runtime
./tools/stop_cameras.sh
./tools/camera_serial.sh
lsusb | grep 2bc5
```

正常情况下 `camera_serial.sh` 应看到 `AU1SB3300XB`、`AU1SB3300YB`、`AU1SB33005A` 三个 serial，`lsusb | grep 2bc5` 应看到三组奥比中光设备。

采集前检查三路图像频率：

```bash
cd third_party/cobot_magic_ros_runtime
./tools/check_cameras.sh
```

通过标准：三个 image topic 都存在并有约 30 Hz 频率。没有相机 topic 时，`lerobot-record` 会在第一帧报 `No Cobot Magic ROS image has been received`。

## 4. 低速主从跟随 Smoke Test

默认开启夹爪同步，统一验证 6 个 arm joints 加夹爪的 7D 单臂动作。`teleop.startup_sync=true` 会先把从臂慢速对齐到主臂当前姿态；同步完成后，`teleop.relative_takeover=true` 会重新记录主从姿态，并按主臂增量驱动从臂，避免后续接管跳变。

```bash
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

通过标准：主臂有重力补偿、可自然拖动；启动同步阶段从臂慢速靠近主臂姿态；同步后左右从臂和夹爪跟随方向正确；动作连续，无明显延迟、抖动、回零或通信报错。任一关节或夹爪方向异常时立即停止，先修正 CAN/topic 映射。

## 5. 主从采集

确认跟随后再录制短数据集。默认会采三路相机；只有做纯关节 smoke test 时才加 `--robot.cameras='{}'`。正常数据录制和 HIL 录制都不要覆盖成单相机配置。

开始采集前确认四个机械臂 ROS 终端和 `cobot_magic_cameras_astra` 相机终端都还在运行。`dataset.repo_id` 每次 smoke test 建议换新名字，例如 `local/cobot_magic_ros_smoke_001`，避免和半途失败的数据目录冲突。

当前 Python/ROS 适配层只能向从臂 command topic 发目标关节，主臂 ROS backend 是只读状态源，不能由 Evo-RL 主动驱动主臂归零。这里的 0 位指数据采集的初始物理姿态，不要求 ROS JointState 数值全为 `0.0`。这台机器当前固定初始姿态已写入 `src/lerobot/robots/cobot_magic_ros/reset_poses/cobot_magic_ros_x5_initial_pose.json`；录制和回放测试统一要求从这个初始姿态开始、结束也回到这个初始姿态。

下面命令使用 `--reset_to_zero_pose=true`，程序会读取项目内固定 JSON：`joint_pos` 作为从臂慢速复位目标，`leader_joint_pos` 作为主臂启动检查基准。开始前先把从臂和两条主臂都放回该固定初始姿态；如果主臂读数超过 `reset_pose_tolerance` 会直接报错，避免从错误姿态开始采集。

第一次验证建议只录 1 个 episode，确认 reset 和 startup sync 方向正确后再增加 episode 数：

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
  --reset_to_zero_pose=true \
  --reset_before_record=true \
  --reset_after_episode=true \
  --reset_duration_s=8.0 \
  --reset_pose_tolerance=0.05 \
  --dataset.repo_id=local/cobot_magic_ros_smoke_001 \
  --dataset.single_task="cobot magic smoke test" \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=10 \
  --dataset.reset_time_s=5 \
  --dataset.fps=30 \
  --dataset.push_to_hub=false \
  --display_data=true
```

`reset_before_record=true` 会在每段数据开始前先把从臂慢速拉回 JSON 固定初始姿态；随后 `teleop.startup_sync=true` 会确认主臂也在该固定姿态附近，再重新建立相对遥操作零点。`reset_after_episode=true` 会在每段 episode 结束后把从臂慢速拉回初始姿态。新数据集会保存 `cam_high`、`cam_left_wrist`、`cam_right_wrist` 三路图像。

多 episode 采集时，每段结束后从臂会回到 JSON 固定初始姿态；在下一段开始前，操作员需要把两条主臂也手动放回同一姿态。否则程序会在开始下一段前报错，不会再把从臂从初始姿态同步到主臂停留的当前姿态。中间的 `dataset.reset_time_s` 是给你整理场景和手动摆回主臂的时间；如果不需要这个窗口，可以设为 `0`。

采集后检查：

```bash
lerobot-dataset-report --dataset local/cobot_magic_ros_smoke_001
lerobot-dataset-viz --repo-id local/cobot_magic_ros_smoke_001 --episode-index 0 --display-compressed-images 0
```

通过标准：action 为左右臂关节和夹爪目标，shape 为 14；图像、episode 时长和 action 曲线正常，没有明显断帧或阶跃。旧的 12D 数据集是早期 `sync_gripper=false` smoke 数据，不再作为新训练流程基准，不能和新 14D 数据直接混用。

## 6. 回放验证

第一次回放保持低速空间和急停准备，只回放短 episode。动作回放不需要读取相机；因为上面的采集命令使用了 `--robot.sync_gripper=true`，这里也必须保持一致，否则 action 维度会和数据集不匹配。

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

回放前会先慢速回到 JSON 固定初始姿态；`require_replay_start_pose=true` 会检查数据集第一帧 action 是否接近该固定姿态，避免突然跳到第一帧动作。回放结束后从臂会慢速回到同一初始姿态。

如果需要边回放边看实时相机，先运行 `third_party/cobot_magic_ros_runtime/tools/check_cameras.sh`，确认三路图像都在约 30 Hz，再去掉 `--robot.cameras='{}'`。任一路相机 topic 缺失时，默认 replay 会在 `robot.get_observation()` 阶段失败。

通过标准：从臂动作方向和幅度与采集一致；退出后无高刚度保持或异常拖拽感。

## 7. 真机 RL / Human-in-loop Rollout

有 policy checkpoint 后进入 HIL。按 `i` 切换人工接管，按 `s` 标记成功并结束 episode，按 `f` 标记失败并结束 episode，按 `Esc` 停止。HIL 录制默认保存三路相机；当前单视角 policy 只消费 `cam_left_wrist`，另外两路会写入 dataset 供后续三视角训练使用。

```bash
lerobot-human-inloop-record \
  --robot.type=cobot_magic_ros_follower \
  --robot.id=cobot_magic_ros_follower_hil \
  --teleop.type=cobot_magic_ros_leader \
  --teleop.id=cobot_magic_ros_leader_hil \
  --teleop.relative_takeover=true \
  --policy.path=/path/to/policy/pretrained_model \
  --dataset.repo_id=local/cobot_magic_ros_hil \
  --dataset.single_task="cobot magic hil smoke test" \
  --dataset.num_episodes=3 \
  --dataset.episode_time_s=15 \
  --dataset.reset_time_s=8 \
  --dataset.fps=30 \
  --dataset.push_to_hub=false \
  --display_data=true
```

通过标准：policy 能稳定输出动作；未接管时从臂执行 policy；按 `i` 后主臂增量接管从臂且不跳变；数据集中包含 policy action、intervention 状态和 episode success/failure 标签。

## 8. 失败处理与边界

- 没有 state topic：先查 `tools/can.sh`、udev 映射和四个 ROS 终端是否报错。
- 没有 image topic：先在 `third_party/cobot_magic_ros_runtime` 下运行 `./tools/camera_serial.sh` 和 `./tools/cameras.sh`，再用 `./tools/check_cameras.sh` 确认频率。
- `FileExistsError: .../.cache/huggingface/lerobot/local/<repo_id>`：该 `dataset.repo_id` 已经创建过。换一个新的 `--dataset.repo_id`，或确认旧数据不需要后删除对应目录；半途失败时目录里可能只有 `meta/info.json`。
- command topic 没有 subscriber：从臂节点未启动或没有用 `control_mode:=2`。
- 从臂跳变：确认使用 `cobot_magic_ros_*` 类型，并开启 `teleop.relative_takeover=true`；不要混用旧 SDK 类型。
- 主臂无重力补偿：问题在 ROS/C++ runtime 或 CAN 映射，不在 Evo-RL Python 适配层。
- 在 30 fps、短 episode 稳定前，不要执行长时间 RL rollout 或带硬物任务。
