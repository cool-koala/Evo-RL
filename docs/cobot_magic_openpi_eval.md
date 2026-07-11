# Cobot Magic OpenPI 真机评测

这份说明记录当前已经在真机上验证有效的 OpenPI/pi0.5 远程推理评测链路。默认参数不要随意改动，尤其是 state 格式、左右臂动作顺序和 action chunk 执行方式。

## 前置条件

在本机使用 `evo-rl-ros2-jazzy` 环境，并确保 ROS2 Jazzy 可用：

```bash
cd /home/guoxiaoyu/Evo-RL
conda activate evo-rl-ros2-jazzy
```

远程 OpenPI joint policy server 默认地址：

```text
ws://115.190.52.37:5352
```

本地相机默认映射：

```text
cam_high        -> /camera_f/color/image_raw
cam_left_wrist  -> /camera_l/color/image_raw
cam_right_wrist -> /camera_r/color/image_raw
```

## 启动本地机器人运行时

打开一个终端，启动 Cobot Magic/X5 本地 ROS runtime。Joint policy 评测需要 follower 订阅
`/cobot_magic/command/joint_*`，所以使用 policy 模式：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --mode policy
```

这个终端需要保持打开。脚本会启动/检查机械臂和相机。

从不带双臂模式回执的旧 runtime 升级时，先编译一次再启动：

```bash
cd /home/guoxiaoyu/Evo-RL
third_party/cobot_magic_ros_runtime/remote_control/tools/build.sh
./scripts/cobot_magic_restart_runtime.sh --mode policy
```

如果要评测 EE pose policy，不启动旧 C++ follower，从本仓库启动 ARX5 Cartesian bridge：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_policy.sh control_mode:=ee_pose start_client:=false send_actions:=false
```

EE bridge 订阅 `/cobot_magic/command/ee_left|right`，并继续发布
`/cobot_magic/puppet/joint_left|right` 和 `/cobot_magic/puppet/end_left|right`，所以观测和录制 schema
仍兼容现有 OpenPI/LeRobot 路径。

如果相机已经单独启动，或只想重启机械臂：

```bash
./scripts/cobot_magic_restart_runtime.sh --mode policy --no-cameras
```

只停止本地 runtime：

```bash
./scripts/cobot_magic_restart_runtime.sh --stop-only
```

## 运行前检查

在另一个终端执行：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/eval_cobot_magic_cube_into_drawer_openpi.sh check
```

检查内容包括：

- OpenPI websocket server 是否可连接
- 三路 ROS2 相机 topic 是否有图像帧
- Cobot Magic leader/follower state topic 是否有数据
- 当前控制模式对应的 command topic 是否有 subscriber：joint 为
  `/cobot_magic/command/joint_left|right`，EE 为 `/cobot_magic/command/ee_left|right`

Joint HIL 还需要额外确认 `/cobot_magic/leader/manual_control_status_left|right` 各有一个 publisher。
`cobot_magic_restart_runtime.sh` 会自动检查；手动检查可执行：

```bash
ros2 topic info /cobot_magic/leader/manual_control_status_left
ros2 topic info /cobot_magic/leader/manual_control_status_right
```

如果相机已经在运行，并且不希望脚本重启相机：

```bash
SKIP_CAMERA_RESTART=true ./scripts/eval_cobot_magic_cube_into_drawer_openpi.sh check
```

## 一键真机评测

推荐命令：

```bash
EVAL_ID=openpi_cube_005 \
NUM_EPISODES=5 \
SKIP_CAMERA_RESTART=true \
./scripts/eval_cobot_magic_cube_into_drawer_openpi.sh hil
```

评测数据会保存到：

```text
data/eval_openpi_cube_005/lerobot
```

每个 episode 结束时需要打标签。默认按键：

```text
s: success
f: failure
i: intervention toggle
Esc: stop early
```

`i` 的用法没有变化：按一次进入接管，再按一次退出，不要长按。进入时 client 会先等待新鲜的左右
主臂、从臂 joint/EE 状态，然后等待两侧主臂都确认重力补偿模式；任一侧超时都会取消接管。退出时
先用当前主臂位置原子切回位置控制，再返回 policy。若日志出现 `manual-control mode was not
acknowledged` 或 `Stale Cobot Magic ROS`，应停止本轮评测、检查 runtime/topic，不能继续反复按键。

默认安全超时为主臂/从臂状态 `0.5s`、相机 `1.0s`。设置为 `0` 可以关闭检查，但不应用于真机评测。
进程正常退出或异常退出都会尝试关闭人工模式；仍应保持急停可用，并在空载状态先重复验证进入/退出。

如果只是检查远程推理和本地观测，不发送动作：

```bash
SEND_ACTIONS=false RECORD=false ./scripts/eval_cobot_magic_cube_into_drawer_openpi.sh dry-run
```

EE pose policy dry-run：

```bash
CONTROL_MODE=ee_pose \
SEND_ACTIONS=false \
RECORD=false \
EE_PORT=5353 \
./scripts/eval_cobot_magic_cube_into_drawer_openpi.sh dry-run
```

真实 EE pose 评测必须显式打开动作：

```bash
CONTROL_MODE=ee_pose \
SEND_ACTIONS=true \
RECORD=true \
EE_PORT=5353 \
./scripts/eval_cobot_magic_cube_into_drawer_openpi.sh hil
```

## 当前有效默认参数

`scripts/eval_cobot_magic_cube_into_drawer_openpi.sh` 已经把当前有效参数设为默认：

```text
HOST=115.190.52.37
JOINT_PORT=5352
EE_PORT=5353
CONTROL_MODE=joint
INPUT_FORMAT=aloha
OPENPI_STATE_FORMAT=observation56
IMAGE_SIZE=224
ACTION_HORIZON=16
PREFETCH_REMAINING_STEPS=0
POLICY_RELATIVE_LIMIT=false
SWAP_JOINT_ARMS=true
```

关键点：

- `OPENPI_STATE_FORMAT=observation56`：当前远程 server 期望客户端发送完整 56 维 `observation.state`，server 端再选择 joint position 维度。
- `SWAP_JOINT_ARMS=true`：当前远程 server 返回的 14 维 joint action 是 `right arm 7 + left arm 7`，本地 Cobot Magic action schema 是 `left arm 7 + right arm 7`，客户端必须交换后再发给机器人。
- `ACTION_HORIZON=16` 和 `PREFETCH_REMAINING_STEPS=0`：每次完整执行一个 16 步 action chunk，不提前切换到下一段 chunk。
- `POLICY_RELATIVE_LIMIT=false`：OpenPI policy action 默认不走机器人相对动作限幅，避免把模型输出截断成不符合训练分布的动作。
- `CONTROL_MODE=joint|ee_pose`：选择真实执行 joint 14D action 还是 EE 14D action。EE action 是绝对
  `x,y,z,wx,wy,wz,gripper`，其中 `wx/wy/wz` 是 ARX 6D pose 后三维，不是 ROS quaternion。

## 依赖说明

EE bridge 依赖 `arx5-interface`：

```bash
cd /home/guoxiaoyu/Evo-RL
pip install -e ".[cobot_magic]"
```

这个包包含 Cartesian controller 和 IK solver；默认不把 `real-stanford/arx5-sdk` 作为 git submodule
放进 Evo-RL。

## 图像处理

客户端会把三路 ROS RGB 图像按 OpenPI client 的 `resize_with_pad(224, 224)` 处理后发送给 server。这里使用的是 OpenPI client 的 PIL resize-with-padding 路径，不是 OpenCV resize。

不要在客户端额外改 RGB/BGR、裁剪或相机顺序。当前图像顺序和训练数据对齐：

```text
cam_high, cam_left_wrist, cam_right_wrist
```

## 常见问题

### 相机等待失败

如果看到：

```text
No image frame received from /camera_f/color/image_raw
```

先检查相机：

```bash
third_party/cobot_magic_ros_runtime/tools/check_cameras.sh
```

必要时重启相机：

```bash
third_party/cobot_magic_ros_runtime/tools/cameras.sh
```

如果相机已经正常运行，评测时加：

```bash
SKIP_CAMERA_RESTART=true
```

### 动作抖动或没有执行完整 chunk

确认不要改掉这两个默认值：

```text
ACTION_HORIZON=16
PREFETCH_REMAINING_STEPS=0
```

需要打印每个 policy step：

```bash
LOG_POLICY_STEPS=true ./scripts/eval_cobot_magic_cube_into_drawer_openpi.sh hil
```

### 左右臂动作反了

当前 server raw action 顺序是 `right 7 + left 7`。本地脚本默认 `SWAP_JOINT_ARMS=true`，会把它转换成本地需要的 `left 7 + right 7`。

只有在远程 server 的训练/部署配置改成 left-first 后，才应该使用：

```bash
SWAP_JOINT_ARMS=false ./scripts/eval_cobot_magic_cube_into_drawer_openpi.sh hil
```

### 临时关闭真实动作

不要用真机动作时：

```bash
SEND_ACTIONS=false RECORD=false ./scripts/eval_cobot_magic_cube_into_drawer_openpi.sh dry-run
```

## 不要提交的目录

以下目录用于本地依赖、训练数据、评测数据或模型输出，已经通过 `.gitignore` 忽略：

```text
openpi/
data/
outputs/
```
