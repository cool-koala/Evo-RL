from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("control_mode", default_value="0"),
        DeclareLaunchArgument("joint_topic", default_value="/cobot_magic/leader/joint_left"),
        DeclareLaunchArgument("state_topic", default_value="/cobot_magic/puppet/joint_left"),
        DeclareLaunchArgument("command_topic", default_value="/cobot_magic/leader/command_joint_left"),
        DeclareLaunchArgument("manual_control_topic", default_value="/cobot_magic/leader/manual_control_left"),
        DeclareLaunchArgument("end_topic", default_value="/cobot_magic/leader/end_left"),
    ]
    return LaunchDescription(
        args
        + [
            Node(
                package="arm_control",
                executable="arm_node",
                name="arm_node3",
                output="screen",
                parameters=[
                    {
                        "control_mode": LaunchConfiguration("control_mode"),
                        "joint_topic": LaunchConfiguration("joint_topic"),
                        "state_topic": LaunchConfiguration("state_topic"),
                        "command_topic": LaunchConfiguration("command_topic"),
                        "manual_control_topic": LaunchConfiguration("manual_control_topic"),
                        "end_topic": LaunchConfiguration("end_topic"),
                    }
                ],
            )
        ]
    )
