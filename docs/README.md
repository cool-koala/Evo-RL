# Cobot Magic/X5 现场操作命令

这份文档只放现场要复制运行的命令。`docs/source/` 是上游 LeRobot 网站文档，现场不用看。

## 1. 一键重启 ROS2 机械臂 + 相机

fish、bash、zsh 都直接执行，不要 `source`：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh
```

默认是 policy/HIL/OpenPI 模式：

- 自动停止旧机械臂节点。
- 自动停止旧相机节点。
- 自动杀掉旧 `slcand` / `slcan_attach`。
- 自动启动三路相机。
- 自动等待三路相机出帧。
- 自动启动 CAN 和四条机械臂 ROS2 节点。
- 自动检查四个 arm state topic。
- 自动检查 `/cobot_magic/command/joint_left|right` 有 subscriber。
- 自动检查左右主臂 `manual_control_status` publisher 存在。

这个终端要保持打开。按 `Ctrl-C` 会停止 arm runtime。

从旧版本升级后第一次使用，需要先重新编译 runtime：

```bash
cd /home/guoxiaoyu/Evo-RL
third_party/cobot_magic_ros_runtime/remote_control/tools/build.sh
```

只改 Python 参数或文档时不需要重复编译；更新 `master1/master2` 的 C++ 控制代码后必须重新编译。

## 2. 只停止所有硬件 runtime

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --stop-only
```

## 3. 常用启动模式

policy/HIL/OpenPI 模式，默认就是这个：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --mode policy
```

普通主从遥操作模式：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --mode leader
```

只重启机械臂，不动相机：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --no-cameras
```

只重启相机，不动机械臂：

```bash
cd /home/guoxiaoyu/Evo-RL/third_party/cobot_magic_ros_runtime
./tools/cameras.sh
./tools/wait_cameras.sh 20
```

## 4. 回初始位

`Ctrl-C` 退出 OpenPI runner 后，机械臂会停在最后一次 command 的位置并继续保持力矩。底层 runtime
还开着时，直接执行：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_reset_pose.sh
```

慢一点回去：

```bash
./scripts/cobot_magic_reset_pose.sh --duration-s 12 --fps 30
```

只回从臂，不动主臂：

```bash
./scripts/cobot_magic_reset_pose.sh --no-leader
```

如果你按 `Ctrl-C` 停掉的是 `./scripts/cobot_magic_restart_runtime.sh` 那个终端，先重新启动 runtime：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --mode policy
```

然后另开终端执行 `./scripts/cobot_magic_reset_pose.sh`。

## 5. 已有底层脚本

这些脚本本来就有，我新增的一键脚本只是把它们串起来：

| 功能 | 脚本 |
| --- | --- |
| 停止机械臂 ROS2 节点 | `third_party/cobot_magic_ros_runtime/remote_control/tools/stop_arms.sh` |
| 启动/刷新 CAN | `third_party/cobot_magic_ros_runtime/remote_control/tools/can.sh` |
| 启动四条机械臂 | `third_party/cobot_magic_ros_runtime/remote_control/tools/remote.sh` |
| 查看 CANable 映射 | `third_party/cobot_magic_ros_runtime/remote_control/tools/arm_serial.sh` |
| 停止相机 | `third_party/cobot_magic_ros_runtime/tools/stop_cameras.sh` |
| 启动相机 | `third_party/cobot_magic_ros_runtime/tools/cameras.sh` |
| 等待相机出帧 | `third_party/cobot_magic_ros_runtime/tools/wait_cameras.sh` |
| 检查相机 topic | `third_party/cobot_magic_ros_runtime/tools/check_cameras.sh` |

`remote.sh` 默认是普通主从模式。policy/HIL/OpenPI 必须用：

```bash
FOLLOWER_INPUT=policy ./tools/remote.sh
```

一键脚本默认已经帮你设置成 policy 模式。

## 6. 手动检查 ROS2 话题

```bash
cd /home/guoxiaoyu/Evo-RL
source ~/anaconda3/etc/profile.d/conda.sh
conda activate evo-rl-ros2-jazzy
source /opt/ros/jazzy/setup.bash

ros2 topic list | rg "/cobot_magic/(leader|puppet|command)"
```

检查四个机械臂状态 topic：

```bash
for topic in \
  /cobot_magic/leader/joint_left \
  /cobot_magic/leader/joint_right \
  /cobot_magic/puppet/joint_left \
  /cobot_magic/puppet/joint_right
do
  printf "%s ... " "$topic"
  timeout 5s ros2 topic echo --once "$topic" >/dev/null && echo ok || echo missing
done
```

policy/HIL/OpenPI 模式还要检查 command subscriber：

```bash
ros2 topic info /cobot_magic/command/joint_left
ros2 topic info /cobot_magic/command/joint_right
```

HIL 还必须有左右主臂模式回执 publisher：

```bash
ros2 topic info /cobot_magic/leader/manual_control_status_left
ros2 topic info /cobot_magic/leader/manual_control_status_right
```

两条命令都应显示 `Publisher count: 1`。进入或退出接管时，可以另开终端观察实际回执：

```bash
# 终端 A
ros2 topic echo /cobot_magic/leader/manual_control_status_left

# 终端 B
ros2 topic echo /cobot_magic/leader/manual_control_status_right
```

不要在 client 尚未发出切换请求时用 `--once` 等待回执；该状态在模式命令到达时发布，不是固定频率
心跳。

## 7. CAN 映射

| CAN | 设备 | 位置 | 角色 | 状态 topic |
| --- | --- | --- | --- | --- |
| `can0` | `/dev/canable0` | 右后 | 右主臂 | `/cobot_magic/leader/joint_right` |
| `can1` | `/dev/canable1` | 右前 | 右从臂 | `/cobot_magic/puppet/joint_right` |
| `can2` | `/dev/canable2` | 左后 | 左主臂 | `/cobot_magic/leader/joint_left` |
| `can3` | `/dev/canable3` | 左前 | 左从臂 | `/cobot_magic/puppet/joint_left` |

检查：

```bash
cd /home/guoxiaoyu/Evo-RL/third_party/cobot_magic_ros_runtime/remote_control
./tools/arm_serial.sh
for iface in can0 can1 can2 can3; do ip -details link show "$iface"; done
```

## 8. 相机

| 视角 | serial | ROS2 topic | LeRobot key |
| --- | --- | --- | --- |
| 前视 | `AU1SB3300XB` | `/camera_f/color/image_raw` | `cam_high` |
| 左腕 | `AU1SB3300YB` | `/camera_l/color/image_raw` | `cam_left_wrist` |
| 右腕 | `AU1SB33005A` | `/camera_r/color/image_raw` | `cam_right_wrist` |

检查：

```bash
cd /home/guoxiaoyu/Evo-RL/third_party/cobot_magic_ros_runtime
./tools/camera_serial.sh
./tools/check_cameras.sh
```

## 9. 主从 smoke test

先用主从模式启动 runtime：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --mode leader
```

另开终端：

```bash
cd /home/guoxiaoyu/Evo-RL
source ~/anaconda3/etc/profile.d/conda.sh
conda activate evo-rl-ros2-jazzy
source /opt/ros/jazzy/setup.bash

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

## 10. 采集数据

先用主从模式启动 runtime：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --mode leader
```

另开终端直接用采集脚本：

```bash
cd /home/guoxiaoyu/Evo-RL
TASK_ID=cobot_magic_cube_into_drawer_v1 \
TASK_DESC="put cube in drawer" \
N=50 \
./scripts/record_cobot_magic_cube_into_drawer.sh
```

按键：

- `s`: 成功并保存当前 episode。
- `f`: 失败并保存当前 episode。
- 左方向键：丢弃当前 episode 重录。
- `Esc`: 停止采集。

## 11. OpenPI 远程 joint policy

先用 policy 模式启动 runtime：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --mode policy
```

真实控制只连接一种策略。当前机械臂执行 joint 控制，所以只连 joint server：

- joint policy: `115.190.52.37:5352`
- EE pose server `115.190.52.37:5353` 不用于真实控制

另开终端，先 dry-run，不移动机械臂：

```bash
cd /home/guoxiaoyu/Evo-RL
source ~/anaconda3/etc/profile.d/conda.sh
conda activate evo-rl-ros2-jazzy
source /opt/ros/jazzy/setup.bash

python scripts/openpi_cobot_magic_hil.py \
  --dry-run-steps 5 \
  --no-record \
  --no-send-actions \
  --request-timeout-s 30 \
  --prefetch-remaining-steps 8 \
  --openpi-state-format observation56 \
  --no-mirror-policy-to-leader \
  --host 115.190.52.37 \
  --joint-port 5352
```

真实执行：

```bash
python scripts/openpi_cobot_magic_hil.py \
  --send-actions \
  --record \
  --force-overwrite \
  --request-timeout-s 30 \
  --prefetch-remaining-steps 8 \
  --openpi-state-format observation56 \
  --host 115.190.52.37 \
  --joint-port 5352 \
  --reset-pose-path src/lerobot/robots/cobot_magic_ros/reset_poses/cobot_magic_ros_x5_initial_pose.json \
  --reset-before-episode \
  --reset-after-episode \
  --reset-on-exit \
  --reset-duration-s 8 \
  --repo-id local/openpi_cobot_magic_hil \
  --dataset-root data/openpi_cobot_magic_hil/lerobot \
  --num-episodes 10 \
  --episode-time-s 60 \
  --task "put cube in drawer"
```

`observation56` 是当前远程 Cobot Magic OpenPI server 需要的 state 格式。服务端会从 56D
`observation.state` 里选择 joint position。不要用旧的 14D state，否则服务端会报
`IndexError: index 21 is out of bounds for axis 0 with size 14`。

默认只驱动从臂，不再把 policy action 镜像发给主臂。需要保持主臂完全不动时，保留
`--no-mirror-policy-to-leader`。server 返回的是 action chunk，例如 `(16, 14)`，也就是 16 步、
每步 14 维 joint action。客户端按控制频率每个循环执行 chunk 里的下一个 action。为了减少
chunk 边界卡顿，`--prefetch-remaining-steps 8` 会在当前 chunk 还剩 8 步时后台请求下一包。

人工接管键仍为 `i`：按一次进入，再按一次退出。切换现在必须收到左右主臂一致回执，失败会直接
报错并执行安全清理，不会继续推进 HIL 状态机。退出时主臂先锁定当前位置，再恢复 policy；不要在
模式切换期间重复按 `i` 或同时执行 `cobot_magic_reset_pose.sh`。ROS joint/EE 状态默认超过 `0.5s`
未更新即拒绝控制，相机超过 `1.0s` 未更新即拒绝使用旧帧。使用 parked HIL recording 状态机时，
退出后才会按其 return-duration 配置缓慢返回主臂初始位。

如果上一次失败已经留下了 `data/openpi_cobot_magic_hil/lerobot/meta/info.json`，继续用同一个
`--dataset-root` 时要保留上面的 `--force-overwrite`，或者换一个新的 `--repo-id` 和
`--dataset-root`。

如果只是调试 EE pose server，可以显式加 shadow 选项：

```bash
--enable-ee-shadow --ee-port 5353
```

这只会记录 EE shadow action，不会把 EE pose 动作发给机械臂。

## 12. OpenPI 远程 EE pose policy

EE pose 控制不使用旧 C++ follower runtime 抢占从臂 CAN。依赖通过 Evo-RL 的 `cobot_magic`
extra 安装：

```bash
cd /home/guoxiaoyu/Evo-RL
pip install -e ".[cobot_magic]"
```

这个安装会拉取 `arx5-interface`，其中包含 `Arx5CartesianController`、`EEFState` 和
`Arx5Solver.inverse_kinematics`。默认不要把 `real-stanford/arx5-sdk` 作为子项目提交进仓库；
只有需要改 SDK 源码时才在本机单独 clone 并 editable install。

先启动本机 EE bridge，首次只 dry-run：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_policy.sh control_mode:=ee_pose start_client:=false send_actions:=false
```

另开终端检查状态和命令 topic：

```bash
ros2 topic echo --once /cobot_magic/puppet/end_left
ros2 topic echo --once /cobot_magic/puppet/end_right
ros2 topic info /cobot_magic/command/ee_left
ros2 topic info /cobot_magic/command/ee_right
```

连接远程 EE policy server 但不移动机械臂：

```bash
cd /home/guoxiaoyu/Evo-RL
source ~/anaconda3/etc/profile.d/conda.sh
conda activate evo-rl-ros2-jazzy
source /opt/ros/jazzy/setup.bash

CONTROL_MODE=ee_pose \
SEND_ACTIONS=false \
RECORD=false \
EE_PORT=5353 \
./scripts/eval_cobot_magic_cube_into_drawer_openpi.sh dry-run
```

真实执行必须显式开启动作：

```bash
CONTROL_MODE=ee_pose \
SEND_ACTIONS=true \
RECORD=true \
EE_PORT=5353 \
./scripts/eval_cobot_magic_cube_into_drawer_openpi.sh hil
```

EE action 是绝对 7D pose：`x,y,z,wx,wy,wz,gripper`。`wx/wy/wz` 沿用当前 ARX runtime
的 6D pose 后三维语义，不是 ROS quaternion。数据集仍保存 28D action，其中 joint 维度由当前观测填充，
EE 维度来自 policy。

## 13. 一键 policy launcher

`scripts/cobot_magic_policy.sh` 是本机一键入口：

```bash
# joint 模式：启动旧 ROS follower runtime
./scripts/cobot_magic_policy.sh control_mode:=joint start_client:=false

# EE 模式：启动相机和 ARX5 Cartesian bridge
./scripts/cobot_magic_policy.sh control_mode:=ee_pose start_client:=false send_actions:=false

# EE 模式同时启动本地 OpenPI client dry-run
./scripts/cobot_magic_policy.sh control_mode:=ee_pose start_client:=true send_actions:=false
```

如果环境已经 source 好，也可以用 launch 文件：

```bash
ros2 launch launch/cobot_magic_policy.launch.py control_mode:=ee_pose start_client:=false send_actions:=false
```

## 14. 常见问题

没有 `/cobot_magic` topic：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --mode policy
```

command topic 没有 subscriber：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --mode policy
```

按 `i` 后提示双臂模式未确认，或只有一侧主臂进入重力补偿：

```bash
ros2 topic info /cobot_magic/leader/manual_control_status_left
ros2 topic info /cobot_magic/leader/manual_control_status_right
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --mode policy
```

不要绕过回执检查继续操作。若刚更新过 C++ runtime，先执行本页第 1 节的 `tools/build.sh`，再重启。
出现 `Stale Cobot Magic ROS ...` 表示对应 joint、EE 或相机 topic 已停止更新，应先恢复 topic，不能把
超时参数调大后继续真机操作。

普通主从不跟随：

```bash
cd /home/guoxiaoyu/Evo-RL
./scripts/cobot_magic_restart_runtime.sh --mode leader
```

相机没有图像：

```bash
cd /home/guoxiaoyu/Evo-RL/third_party/cobot_magic_ros_runtime
./tools/stop_cameras.sh
./tools/camera_serial.sh
./tools/cameras.sh
./tools/wait_cameras.sh 20
```
