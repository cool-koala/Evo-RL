# Dobot Xtrainer 云端算法接入接口说明

本文档面向云端算法开发，说明如何把 `Dobot Xtrainer` 通过 Evo-RL 现有的 `async_inference` 链路接到远端策略服务。

## 1. 总体架构

推荐采用下面这条链路：

1. **边端机器**连接真实 Dobot 机械臂与相机
2. 边端运行 [robot_client.py](/home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/src/lerobot/async_inference/robot_client.py)
3. 云端运行你们自己的 gRPC 策略服务
4. 边端发送 observation
5. 云端返回 action chunk
6. 边端执行 action，并继续回传 observation

这条链路的优点是：

- 机械臂驱动、相机访问、人工介入按钮、灯色、同步、保护逻辑都在边端
- 云端只负责“给动作”
- 不需要让云端直接接机械臂驱动

## 2. 本次已接好的边端能力

### 机器人类型

边端 `async_inference` 已支持：

- `dobot_xtrainer_follower`

对应代码：

- [dobot_xtrainer_follower.py](/home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/src/lerobot/robots/dobot_xtrainer_follower/dobot_xtrainer_follower.py)
- [robot_client.py](/home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/src/lerobot/async_inference/robot_client.py)
- [custom_policy_server.py](/home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/src/lerobot/async_inference/custom_policy_server.py)

### 边端保护

边端已经保留或实现：

- follower 关节步长限幅
- `EE pose` 平移/旋转限幅
- leader 同步/镜像保护
- 按钮触发人工介入
- leader 传感器急停
- 红黄绿灯状态控制

注意：

- 当前 `async_inference` 链路里，云端网络超时后的专门“超时降级状态机”还没有额外实现
- 当动作队列耗尽时，边端不会继续下发新动作，机械臂会保持最后一次已执行命令对应的状态

## 3. 边端启动方式

先设置 `PYTHONPATH`：

```bash
export PYTHONPATH=/home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/src:/home/abc/guoxiaoyu/Dobot_Xtrainer/dobot_xtrainer-master
```

### 3.1 joint-space 云端策略

云端输出 14 维 joint action 时：

```bash
python -m lerobot.async_inference.robot_client \
  --robot.type=dobot_xtrainer_follower \
  --robot.dobot_root=/home/abc/guoxiaoyu/Dobot_Xtrainer/dobot_xtrainer-master \
  --robot.action_mode=joint \
  --server_address=<cloud_host>:8080 \
  --policy_type=my_joint_policy \
  --pretrained_name_or_path=dummy \
  --policy_device=cpu \
  --client_device=cpu \
  --actions_per_chunk=8 \
  --fps=30 \
  --task="pick the cube"
```

### 3.2 EE pose 云端策略

云端输出 16 维末端位姿 action 时：

```bash
python -m lerobot.async_inference.robot_client \
  --robot.type=dobot_xtrainer_follower \
  --robot.dobot_root=/home/abc/guoxiaoyu/Dobot_Xtrainer/dobot_xtrainer-master \
  --robot.action_mode=ee_pose \
  --server_address=<cloud_host>:8080 \
  --policy_type=my_ee_policy \
  --pretrained_name_or_path=dummy \
  --policy_device=cpu \
  --client_device=cpu \
  --actions_per_chunk=8 \
  --fps=30 \
  --task="pick the cube"
```

## 4. 传输协议说明

当前协议基于：

- gRPC service：`AsyncInference`
- protobuf 定义：[services.proto](/home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/src/lerobot/transport/services.proto)
- 实际 observation/action payload：**Python pickle**

这意味着：

- 当前云端接入默认是 **Python-to-Python**
- 不推荐直接用别的语言实现服务端
- 边端与云端最好使用兼容的 Python 主版本

如果后续你们要做跨语言或长期稳定协议，再把 observation/action 换成显式 protobuf/JSON schema 更合适。

## 5. gRPC 方法

### `Ready(Empty) -> Empty`

边端连上后调用，用于重置服务端状态。

### `SendPolicyInstructions(PolicySetup) -> Empty`

边端发送一份 `RemotePolicyConfig`，目前包含：

- `policy_type`
- `pretrained_name_or_path`
- `lerobot_features`
- `actions_per_chunk`
- `device`
- `rename_map`
- `action_feature_names`
- `robot_type`
- `action_mode`

其中最关键的是：

- `action_feature_names`
- `action_mode`

云端可以据此知道应该输出什么维度、什么顺序的 action。

### `SendObservations(stream Observation) -> Empty`

边端不断发送 observation。

### `GetActions(Empty) -> Actions`

云端返回一个 action chunk。

## 6. Observation 接口

云端算法拿到的是 `TimedObservation`，定义在：

- [helpers.py](/home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/src/lerobot/async_inference/helpers.py)

关键字段：

- `timestamp`
- `timestep`
- `observation`
- `must_go`

其中 `observation` 是一个 Python 字典。

### 6.1 observation 字段

当前 `dobot_xtrainer_follower` 会发送：

- `left_joint_1.pos` 到 `left_joint_6.pos`
- `left_gripper.pos`
- `right_joint_1.pos` 到 `right_joint_6.pos`
- `right_gripper.pos`
- `left_ee.x`
- `left_ee.y`
- `left_ee.z`
- `left_ee.qx`
- `left_ee.qy`
- `left_ee.qz`
- `left_ee.qw`
- `right_ee.x`
- `right_ee.y`
- `right_ee.z`
- `right_ee.qx`
- `right_ee.qy`
- `right_ee.qz`
- `right_ee.qw`
- `cam_high`
- `cam_left_wrist`
- `cam_right_wrist`
- `task`

### 6.2 图像格式

- `cam_high`: `np.ndarray`, shape `(480, 640, 3)`
- `cam_left_wrist`: `np.ndarray`, shape `(480, 640, 3)`
- `cam_right_wrist`: `np.ndarray`, shape `(480, 640, 3)`
- dtype 一般是 `uint8`
- 通道顺序是 RGB

## 7. Action 接口

边端 `robot_client` 会按 `robot.action_features` 的顺序解释动作张量。

### 7.1 joint-space 动作

当 `--robot.action_mode=joint` 时，云端必须输出 **14 维**，顺序如下：

1. `left_joint_1.pos`
2. `left_joint_2.pos`
3. `left_joint_3.pos`
4. `left_joint_4.pos`
5. `left_joint_5.pos`
6. `left_joint_6.pos`
7. `left_gripper.pos`
8. `right_joint_1.pos`
9. `right_joint_2.pos`
10. `right_joint_3.pos`
11. `right_joint_4.pos`
12. `right_joint_5.pos`
13. `right_joint_6.pos`
14. `right_gripper.pos`

单位：

- 关节：弧度
- 夹爪：`0~1`

### 7.2 EE pose 动作

当 `--robot.action_mode=ee_pose` 时，云端必须输出 **16 维**，顺序如下：

1. `left_ee.x`
2. `left_ee.y`
3. `left_ee.z`
4. `left_ee.qx`
5. `left_ee.qy`
6. `left_ee.qz`
7. `left_ee.qw`
8. `left_gripper.pos`
9. `right_ee.x`
10. `right_ee.y`
11. `right_ee.z`
12. `right_ee.qx`
13. `right_ee.qy`
14. `right_ee.qz`
15. `right_ee.qw`
16. `right_gripper.pos`

单位：

- 平移：米
- 姿态：四元数 `xyzw`
- 夹爪：`0~1`

## 8. 云端自定义服务接口

为了接入非 LeRobot 策略，仓内新增了：

- [custom_policy_server.py](/home/abc/guoxiaoyu/Dobot_Xtrainer/Evo-RL/src/lerobot/async_inference/custom_policy_server.py)

你们只需要实现一个 adapter。

### 8.1 Adapter 需要实现的方法

```python
class MyPolicyAdapter:
    def setup(self, policy_config):
        ...

    def predict_action_chunk(self, observation, policy_config):
        ...

    def reset(self):
        ...
```

### 8.2 `setup(policy_config)`

用于读取：

- action 维度
- action 名称顺序
- robot 类型
- action mode

### 8.3 `predict_action_chunk(observation, policy_config)`

输入：

- `observation`: `TimedObservation`
- `policy_config`: `RemotePolicyConfig`

返回值允许两种形式：

1. `torch.Tensor` 或 `numpy.ndarray`
   - shape `(action_dim,)`
   - 或 shape `(chunk_size, action_dim)`

2. `list[TimedAction]`
   - 当你们要自己控制每个动作的 `timestamp/timestep` 时使用

如果返回 tensor/ndarray，服务端会自动按当前 observation 的 `timestamp/timestep` 补齐时间信息。

### 8.4 最小示例

```python
import torch

from lerobot.async_inference.configs import PolicyServerConfig
from lerobot.async_inference.custom_policy_server import BaseCloudPolicyAdapter, serve_with_adapter


class ZeroPolicyAdapter(BaseCloudPolicyAdapter):
    def setup(self, policy_config):
        self.action_dim = len(policy_config.action_feature_names)

    def predict_action_chunk(self, observation, policy_config):
        return torch.zeros(1, self.action_dim, dtype=torch.float32)


if __name__ == "__main__":
    server = serve_with_adapter(
        ZeroPolicyAdapter(),
        PolicyServerConfig(host="0.0.0.0", port=8080, fps=30, inference_latency=1 / 30),
    )
    server.wait_for_termination()
```

## 9. 云端开发建议

### 建议 1

服务端直接使用 `policy_config.action_feature_names`，不要手写维度假设。

### 建议 2

如果你们算法只支持 joint-space，就在边端明确启动：

- `--robot.action_mode=joint`

如果算法输出末端位姿，就启动：

- `--robot.action_mode=ee_pose`

### 建议 3

图像预处理、缩放、裁剪、tokenization 放在云端 adapter 内做，不要在边端另加一套特化逻辑。

### 建议 4

云端算法应尽量输出小 chunk，例如 `4~16` 个动作，减少公网延迟下的大量过期动作。

## 10. 当前限制

- 当前 transport payload 是 pickle，不是跨语言协议
- 当前 `async_inference` 链路未单独实现“网络超时 -> 自动切回人工接管”的新状态机
- 云端服务默认不直接感知 leader 按钮事件；按钮和人工介入逻辑仍在边端
- 如果要把 `Cobot Magic` 也纳入同一云端接口，后续只需要让它实现相同的 observation/action 约定
