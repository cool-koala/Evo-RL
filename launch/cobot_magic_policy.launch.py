#!/usr/bin/env python

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description():
    repo_root = Path(__file__).resolve().parents[1]
    control_mode = LaunchConfiguration("control_mode")
    start_cameras = LaunchConfiguration("start_cameras")
    start_client = LaunchConfiguration("start_client")
    send_actions = LaunchConfiguration("send_actions")
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    host = LaunchConfiguration("host")
    joint_port = LaunchConfiguration("joint_port")
    ee_port = LaunchConfiguration("ee_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("control_mode", default_value="joint"),
            DeclareLaunchArgument("start_cameras", default_value="true"),
            DeclareLaunchArgument("start_client", default_value="false"),
            DeclareLaunchArgument("send_actions", default_value="false"),
            DeclareLaunchArgument("left_interface", default_value="can3"),
            DeclareLaunchArgument("right_interface", default_value="can1"),
            DeclareLaunchArgument("host", default_value="115.190.52.37"),
            DeclareLaunchArgument("joint_port", default_value="5352"),
            DeclareLaunchArgument("ee_port", default_value="5353"),
            ExecuteProcess(
                cmd=[
                    "bash",
                    str(repo_root / "scripts/cobot_magic_policy.sh"),
                    ["control_mode:=", control_mode],
                    ["start_cameras:=", start_cameras],
                    ["start_client:=", start_client],
                    ["send_actions:=", send_actions],
                    ["left_interface:=", left_interface],
                    ["right_interface:=", right_interface],
                    ["host:=", host],
                    ["joint_port:=", joint_port],
                    ["ee_port:=", ee_port],
                ],
                cwd=str(repo_root),
                output="screen",
            ),
        ]
    )
