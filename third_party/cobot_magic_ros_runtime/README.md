# Cobot Magic ROS Runtime

该目录包含真机 HIL 使用的 Cobot Magic ROS2 Jazzy/C++ 控制 runtime。仓库只跟踪源码；本地生成的
`build/`、`install/`、`log/` 目录不应提交。

## 编译与启动

更新 `master1/master2/follow1/follow2` 源码后执行：

```bash
cd /home/guoxiaoyu/Evo-RL
third_party/cobot_magic_ros_runtime/remote_control/tools/build.sh
```

该命令只编译，不启动机械臂。policy/HIL 模式使用：

```bash
./scripts/cobot_magic_restart_runtime.sh --mode policy
```

## ROS 接口

runtime 在 `/cobot_magic/...` 发布主臂和从臂状态，并订阅 Evo-RL ROS backend 发送的
`/cobot_magic/command/joint_left|right`。相机 workspace 发布默认配置使用的三路 RGB 图像：
`/camera_f/color/image_raw`、`/camera_l/color/image_raw`、`/camera_r/color/image_raw`。

主臂 HIL 还使用以下模式命令和状态回执：

```text
/cobot_magic/leader/manual_control_left
/cobot_magic/leader/manual_control_right
/cobot_magic/leader/manual_control_status_left
/cobot_magic/leader/manual_control_status_right
```

左右主臂节点只在模式实际变化时初始化控制参数；重复命令不会反复重置重力补偿。关节位置命令会在同一
回调中安装首个目标并退出人工模式，然后发布 `false` 回执。上层 client 必须等待左右回执一致，不能
把 topic 存在等同于模式已经切换成功。

完整现场命令见 [docs/README.md](../../docs/README.md)。
