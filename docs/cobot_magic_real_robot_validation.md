# Cobot Magic 真机 RL 验证教程

本文是当前推荐的 Cobot Magic/X5 真机验证 SOP。真机主从、采集、回放和 human-in-loop rollout 使用 ROS2 Jazzy/C++ runtime 提供 CAN、关节控制、限位和重力补偿，Evo-RL 只负责策略推理、动作仲裁和人工介入。ARX5 Python SDK 路径仅保留为诊断入口，见 [`cobot_magic_arx5.md`](cobot_magic_arx5.md)。

## 0. 安全与环境检查

首次验证保持空载、低速、无遮挡，并确保操作员可以立即扶住机械臂或触发急停。不要在跟随稳定前放置任务物体。

```bash
source ~/anaconda3/etc/profile.d/conda.sh
conda activate evo-rl-ros2-jazzy
python -m ruff check \
  src/lerobot/scripts/lerobot_teleoperate.py \
  src/lerobot/scripts/lerobot_record.py \
  src/lerobot/scripts/lerobot_replay.py \
  src/lerobot/scripts/recording_loop.py \
  src/lerobot/scripts/robot_reset.py \
  src/lerobot/robots/cobot_magic_ros \
  src/lerobot/teleoperators/cobot_magic_ros \
  src/lerobot/utils/cobot_magic_ros.py
python -m py_compile \
  src/lerobot/scripts/lerobot_teleoperate.py \
  src/lerobot/scripts/lerobot_record.py
lerobot-teleoperate --help | rg "cobot_magic_ros|startup_sync"
lerobot-record --help | rg "cobot_magic_ros|startup_sync|reset_to_zero_pose"
```

本机使用专用 conda 环境 `evo-rl-ros2-jazzy`，激活后会自动加载 `/opt/ros/jazzy/setup.bash`。如果环境不存在，先执行 `conda create -n evo-rl-ros2-jazzy python=3.12 pip`，再在仓库根目录运行 `python -m pip install -e ".[dev,test,cobot_magic,transformers-dep]" ruff`。

通过标准：Ruff、`py_compile` 和 CLI 类型注册检查通过，`ROS_DISTRO=jazzy`、`ROS_VERSION=2`。这只验证 Evo-RL 适配层，不代表真机运动安全。

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
conda activate evo-rl-ros2-jazzy
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
pgrep -af 'slcand|ros2 launch arm_control|arm_node' || true
for iface in can0 can1 can2 can3; do ip -details link show "$iface" || true; done
./tools/can.sh
for iface in can0 can1 can2 can3; do ip -details link show "$iface"; done
./tools/remote.sh
```

不要在 `tools/can.sh` 失败后继续运行 `tools/remote.sh`；这样只会让四个 ROS arm node 反复报 `Fail to open can*`。

检查 topic：

```bash
ros2 topic hz /cobot_magic/leader/joint_right
ros2 topic hz /cobot_magic/leader/joint_left
ros2 topic hz /cobot_magic/puppet/joint_right
ros2 topic hz /cobot_magic/puppet/joint_left
ros2 topic hz /cobot_magic/leader/end_right
ros2 topic hz /cobot_magic/leader/end_left
ros2 topic hz /cobot_magic/puppet/end_right
ros2 topic hz /cobot_magic/puppet/end_left
ros2 topic info /cobot_magic/command/joint_right
ros2 topic info /cobot_magic/command/joint_left
```

通过标准：四个 state topic 都有稳定频率，两个 command topic 有 subscriber。

## 3. 启动相机 ROS 节点

当前这台机器使用 3 个 RGB 相机，ROS2 相机包通过 OpenCV/V4L2 读取 `/dev/video*` 或
`/dev/v4l/...` 设备，发布 `rgb8` 图像。旧 ROS1 相机启动脚本不是单一奥比中光方案：
`camera_f` 和 `camera_r` 走自定义 `realsense_sdk_bridge`/`pyrealsense2`，`camera_l` 走
`astra_camera`/OpenNI2/`liborbbec.so`。当前接入本机的三路相机序列号均为 `AU...`，USB VID
为 `2bc5`，视频节点以 UVC/Sonix 方式枚举；右腕看不到时优先排查枚举、USB 带宽和像素格式，
不要直接假设是缺少 Orbbec ROS2 SDK。

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
cd third_party/cobot_magic_ros_runtime
./tools/camera_serial.sh
./tools/cameras.sh
```

如果当前 shell 已经在 `third_party/cobot_magic_ros_runtime` 目录内，直接运行 `./tools/camera_serial.sh` 和 `./tools/cameras.sh`，不要再执行相对路径 `cd third_party/cobot_magic_ros_runtime`。

`tools/cameras.sh` 会打开一个新的 `cobot_magic_cameras` 终端运行相机 ROS2 节点。采集期间不要关闭这个终端；如果已经关掉，重新运行 `./tools/cameras.sh`。默认按 `CAMERA_F_SERIAL`、`CAMERA_L_SERIAL`、`CAMERA_R_SERIAL` 查找稳定设备链接；如果现场设备名固定，也可以设置 `CAMERA_F_DEVICE=/dev/video0`、`CAMERA_L_DEVICE=/dev/video1`、`CAMERA_R_DEVICE=/dev/video2`。

默认强制 `CAMERA_FOURCC=MJPG`。本机三路设备都支持 `MJPG 640x480@30`；如果让 OpenCV 使用默认
未压缩格式，三路同时打开时可能出现右腕能打开但读不到帧。需要验证底层能力时运行：

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video3 --list-formats-ext
v4l2-ctl -d /dev/video7 --list-formats-ext
v4l2-ctl -d /dev/video9 --list-formats-ext
```

如果 `camera_serial.sh` 只显示 2 台设备，或 `check_cameras.sh` 显示 `/camera_r/color/image_raw: missing`，说明系统没有枚举到右腕相机 `AU1SB33005A`。先停止旧相机节点，再检查右腕相机 USB 线、USB hub 供电和插口；重新插拔后再次检查：

```bash
cd third_party/cobot_magic_ros_runtime
./tools/stop_cameras.sh
./tools/camera_serial.sh
lsusb | grep 2bc5
```

正常情况下 `camera_serial.sh` 应看到 `AU1SB3300XB`、`AU1SB3300YB`、`AU1SB33005A` 三个 serial，`lsusb | grep 2bc5` 应看到三组奥比中光设备。

奥比中光官方 `OrbbecSDK_ROS2` 仓库可以用代理拉取，例如
`git clone https://gh-proxy.com/https://github.com/orbbec/OrbbecSDK_ROS2.git`。官方 `v2-main`
分支和 Ubuntu 二进制包支持 ROS2 Jazzy，适合新 UVC 系列设备；README 同时说明旧 OpenNI 设备应看
`main` 分支。本机已验证 `ros-jazzy-orbbec-camera` 能安装，`list_ob_devices.sh` 能在 USB 层看到
三组 `2bc5:050e` RGB 和 `2bc5:060e` depth 设备，但 `ros2 run orbbec_camera list_devices_node`
没有列出可管理设备，`gemini_330_series`/`dabai_a` launch 也未发布图像。因此当前默认 runtime 不
切换到官方 SDK；数据采集只需要 640x480 RGB，继续使用已验证的 OpenCV/V4L2 + MJPG 路径。

采集前检查三路图像频率：

```bash
cd third_party/cobot_magic_ros_runtime
./tools/check_cameras.sh
```

通过标准：三个 image topic 都存在并持续出帧。当前相机仍使用 OpenCV/V4L2 方案，图像实际频率不作为严格 30 Hz 门槛；`dataset.fps=30` 主要约束机械臂状态、action 和记录循环。没有相机 topic 时，`lerobot-record` 会在第一帧报 `No Cobot Magic ROS image has been received`。

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

开始采集前确认四个机械臂 ROS2 终端和 `cobot_magic_cameras` 相机终端都还在运行。`dataset.repo_id` 每次 smoke test 建议换新名字，例如 `local/cobot_magic_ros_smoke_001`，避免和半途失败的数据目录冲突。

当前 Python/ROS 适配层只能向从臂 command topic 发目标关节，主臂 ROS backend 是只读状态源，不能由 Evo-RL 主动驱动主臂归零。这里的 0 位指数据采集的初始物理姿态，不要求 ROS JointState 数值全为 `0.0`。这台机器当前固定初始姿态已写入 `src/lerobot/robots/cobot_magic_ros/reset_poses/cobot_magic_ros_x5_initial_pose.json`；录制和回放测试统一要求从这个初始姿态开始、结束也回到这个初始姿态。

精简采集默认不做自动初始位 reset，减少每段开始和结束的等待，也避免机械臂自动复位影响现场摆位。开始采集前人工把主臂、从臂和任务物体放到合适起始姿态；`teleop.startup_sync=true` 仍会建立主从相对接管零点，避免启动瞬间跳变。

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

如果 `dataset.repo_id` 或 `dataset.root` 已存在，`lerobot-record` 会询问是否覆盖本地旧数据。输入 `y` 删除旧目录并重录，直接回车会取消。批量采集时可传 `--dataset.overwrite=true` 跳过确认。新数据集会保存 joint、EE pose 和 `cam_high`、`cam_left_wrist`、`cam_right_wrist` 三路图像。

多 episode 采集时，每段结束后不会自动复位机械臂；中间的 `dataset.reset_time_s` 是给你整理场景、摆物体和手动回到起始姿态的时间。如果不需要这个窗口，可以设为 `0`。

采集后检查：

```bash
lerobot-dataset-report --dataset local/cobot_magic_ros_smoke_001
lerobot-dataset-viz --repo-id local/cobot_magic_ros_smoke_001 --episode-index 0 --display-compressed-images 0
```

通过标准：`action` shape 为 28，包含双臂 joint+gripper 目标 14D 和双臂 EE pose 14D；`observation.state` shape 为 56，包含双臂 joint pos/vel/torque+gripper 42D 和双臂 EE pose 14D；图像、episode 时长和 action 曲线正常，没有明显断帧或阶跃。旧的 12D 数据集是早期 `sync_gripper=false` smoke 数据，不再作为新训练流程基准，不能和新数据直接混用。

EE pose 按 LeRobot EE space 保存为 7 个标量：`ee.x`、`ee.y`、`ee.z`、`ee.wx`、`ee.wy`、`ee.wz`、`ee.gripper_pos`。双臂字段带左右前缀，例如 `left_ee.x`、`right_ee.gripper_pos`。Cobot Magic runtime 的 EE topic 是 `PoseStamped`，但 `orientation.x/y/z` 实际存放 ARX `End_Effector_Pose[3:6]`，这里按 `wx/wy/wz` 保存，不按 ROS 四元数解释；`orientation.w` 保存夹爪位置。

### 5.1 连续采集 50-100 组数据

正式采集前先清掉旧进程，避免旧 ROS topic、CAN 或相机节点占用设备：

```bash
cd /home/guoxiaoyu/Evo-RL
./third_party/cobot_magic_ros_runtime/remote_control/tools/stop_arms.sh
./third_party/cobot_magic_ros_runtime/tools/stop_cameras.sh
sudo pkill -x slcand || true
```

终端 1 启动四条机械臂，并保持窗口打开。结束采集时在这个终端按 `Ctrl-C`，脚本会请求 arm node 退出并下电/解锁电机：

```bash
cd /home/guoxiaoyu/Evo-RL/third_party/cobot_magic_ros_runtime/remote_control
source ~/anaconda3/etc/profile.d/conda.sh
conda activate evo-rl-ros2-jazzy
./tools/remote.sh
```

终端 2 启动三路 RGB 相机并检查频率：

```bash
cd /home/guoxiaoyu/Evo-RL/third_party/cobot_magic_ros_runtime
./tools/cameras.sh
./tools/check_cameras.sh
```

终端 3 从仓库根目录开始采集。`N=50` 可改为 `N=100`；`episode_time_s` 是单组最大时长，完成或失败后用按键提前结束：

```bash
cd /home/guoxiaoyu/Evo-RL
source ~/anaconda3/etc/profile.d/conda.sh
conda activate evo-rl-ros2-jazzy

TASK_ID=cobot_magic_task_001
TASK_DESC="your task name"
N=50

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
  --dataset.repo_id=local/${TASK_ID} \
  --dataset.root=/home/guoxiaoyu/Evo-RL/data/${TASK_ID}/lerobot \
  --dataset.single_task="${TASK_DESC}" \
  --dataset.num_episodes=${N} \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=8 \
  --dataset.fps=30 \
  --dataset.push_to_hub=false \
  --dataset.vcodec=h264 \
  --dataset.overwrite=false \
  --enable_episode_outcome_labeling=true \
  --require_episode_success_label=true \
  --episode_success_key=s \
  --episode_failure_key=f \
  --display_data=false
```

每组 episode 的按键规则：按 `s` 保存为成功并结束当前组，按 `f` 保存为失败并结束当前组，按左方向键丢弃当前组并重录，按 `Esc` 停止整个采集。不要用右方向键结束正式数据，因为当前命令要求必须写入成功/失败标签。标签保存在 episode metadata 的 `episode_success` 字段，取值为 `success` 或 `failure`。

数据保存到 `/home/guoxiaoyu/Evo-RL/data/${TASK_ID}/lerobot`。新数据按 `dataset.fps=30` 记录机械臂状态和 action，包含 `cam_high`、`cam_left_wrist`、`cam_right_wrist` 三路 RGB 图像；相机帧率允许低于 30 Hz。`action` 为 28D，包含双臂 joint+gripper 目标和双臂 EE pose；`observation.state` 为 56D，包含双臂 joint pos/vel/torque+gripper 和双臂 EE pose。

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

如果需要边回放边看实时相机，先运行 `third_party/cobot_magic_ros_runtime/tools/check_cameras.sh`，确认三路图像 topic 都存在并持续出帧，再去掉 `--robot.cameras='{}'`。任一路相机 topic 缺失时，默认 replay 会在 `robot.get_observation()` 阶段失败。

通过标准：从臂动作方向和幅度与采集一致；退出后无高刚度保持或异常拖拽感。

## 7. Diffusion Policy 训练：只使用成功 episode

Diffusion Policy 训练会自动使用数据集的 `meta/stats.json` 做归一化。正式训练不要只在
`lerobot-train` 里临时传 `--dataset.episodes=[...]`，因为这样样本被过滤了，但归一化 stats
仍来自原始全量数据集。推荐先物化一份按 episode 标签过滤后的派生数据集，让训练样本和
`meta/stats.json` 都来自同一批 episode。

下面命令从正式采集数据中筛选 `episode_success=success`，生成 success-only 派生数据集。
默认视频使用 symlink 复用原始 mp4，所以原始数据集目录不能删除；如果要单独搬走派生数据集，
把 `--video-mode symlink` 改成 `--video-mode copy`。

```bash
cd /home/guoxiaoyu/Evo-RL
source ~/anaconda3/etc/profile.d/conda.sh
conda activate lerobot

TASK_ID=cobot_magic_cube_into_drawer_v1

python scripts/lerobot_materialize_episode_filter.py \
  --source-root data/${TASK_ID}/lerobot \
  --output-root data/${TASK_ID}_success_only/lerobot \
  --metadata-key episode_success \
  --metadata-value success \
  --video-mode symlink \
  --overwrite
```

生成后检查 episode 数、帧数和 stats count。`action` 和 `observation.state` 的 `count`
应等于成功 episode 的总帧数：

```bash
python - <<'PY'
from lerobot.datasets.lerobot_dataset import LeRobotDataset

task_id = "cobot_magic_cube_into_drawer_v1"
root = f"data/{task_id}_success_only/lerobot"
dataset = LeRobotDataset(repo_id=f"local/{task_id}_success_only", root=root)
print("episodes", dataset.meta.total_episodes)
print("frames", dataset.meta.total_frames)
print("action count", dataset.meta.stats["action"]["count"])
print("state count", dataset.meta.stats["observation.state"]["count"])
PY
```

训练时直接指向派生数据集，不再传 `--dataset.episodes`：

```bash
lerobot-train \
  --dataset.repo_id=local/${TASK_ID}_success_only \
  --dataset.root=data/${TASK_ID}_success_only/lerobot \
  --policy.type=diffusion \
  --policy.device=cuda \
  --policy.use_amp=true \
  --batch_size=1 \
  --steps=100000 \
  --eval_freq=0 \
  --save_freq=10000 \
  --save_checkpoint=true \
  --wandb.enable=false \
  --policy.push_to_hub=false \
  --num_workers=8 \
  --output_dir=outputs/train/${TASK_ID}_diffusion_success_only_v1
```

如果之后要训练失败分布或混合分布，不需要改 LeRobot 源码。失败-only 数据集把
`--metadata-value` 改成 `failure`，输出到 `data/${TASK_ID}_failure_only/lerobot`；混合训练
直接使用原始数据集，或按自己的规则再生成一个新的派生数据集。关键原则是：训练用哪些 episode，
`meta/stats.json` 就应该由同一批 episode 聚合出来。

`scripts/lerobot_train_episode_filter.py` 只适合快速实验：它会按 episode metadata 转发
`--dataset.episodes` 给原生 `lerobot-train`，但不会重算 `meta/stats.json`。正式训练优先使用
上面的物化数据集流程。

## 8. 真机 RL / Human-in-loop Rollout

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

## 9. 失败处理与边界

- 没有 state topic：先查 `tools/can.sh`、udev 映射和四个 ROS 终端是否报错。
- 没有 image topic：先在 `third_party/cobot_magic_ros_runtime` 下运行 `./tools/camera_serial.sh` 和 `./tools/cameras.sh`，再用 `./tools/check_cameras.sh` 确认频率。
- `FileExistsError: .../.cache/huggingface/lerobot/local/<repo_id>`：该 `dataset.repo_id` 已经创建过。换一个新的 `--dataset.repo_id`，或确认旧数据不需要后删除对应目录；半途失败时目录里可能只有 `meta/info.json`。
- command topic 没有 subscriber：从臂节点未启动或没有用 `control_mode:=2`。
- 从臂跳变：确认使用 `cobot_magic_ros_*` 类型，并开启 `teleop.relative_takeover=true`；不要混用旧 SDK 类型。
- 主臂无重力补偿：问题在 ROS/C++ runtime 或 CAN 映射，不在 Evo-RL Python 适配层。
- 在 30 fps、短 episode 稳定前，不要执行长时间 RL rollout 或带硬物任务。
