# Cobot Magic ROS Runtime

This directory vendors the Cobot Magic ROS2 Jazzy/C++ control runtime used for real robot HIL.
It is kept as source-only runtime code. Colcon outputs such as `build/`, `install/`, and `log/`
are generated locally and should not be committed.

The vendored copy publishes leader and follower state on `/cobot_magic/...` topics and subscribes to
`/cobot_magic/command/joint_left|right`, which is what the Evo-RL ROS backend sends. The camera
workspace publishes the three RGB image topics used by the default Evo-RL config:
`/camera_f/color/image_raw`, `/camera_l/color/image_raw`, and `/camera_r/color/image_raw`.

See `docs/cobot_magic_ros_hil.md` for build and run commands.
