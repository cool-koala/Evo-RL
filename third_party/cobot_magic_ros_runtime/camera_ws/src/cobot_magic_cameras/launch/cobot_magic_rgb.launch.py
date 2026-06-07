from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("width", default_value=EnvironmentVariable("CAMERA_WIDTH", default_value="640")),
        DeclareLaunchArgument("height", default_value=EnvironmentVariable("CAMERA_HEIGHT", default_value="480")),
        DeclareLaunchArgument("fps", default_value=EnvironmentVariable("CAMERA_FPS", default_value="30")),
        DeclareLaunchArgument("fourcc", default_value=EnvironmentVariable("CAMERA_FOURCC", default_value="MJPG")),
        DeclareLaunchArgument("camera_f_device", default_value=EnvironmentVariable("CAMERA_F_DEVICE", default_value="")),
        DeclareLaunchArgument("camera_l_device", default_value=EnvironmentVariable("CAMERA_L_DEVICE", default_value="")),
        DeclareLaunchArgument("camera_r_device", default_value=EnvironmentVariable("CAMERA_R_DEVICE", default_value="")),
        DeclareLaunchArgument(
            "camera_f_serial", default_value=EnvironmentVariable("CAMERA_F_SERIAL", default_value="AU1SB3300XB")
        ),
        DeclareLaunchArgument(
            "camera_l_serial", default_value=EnvironmentVariable("CAMERA_L_SERIAL", default_value="AU1SB3300YB")
        ),
        DeclareLaunchArgument(
            "camera_r_serial", default_value=EnvironmentVariable("CAMERA_R_SERIAL", default_value="AU1SB33005A")
        ),
    ]
    return LaunchDescription(
        args
        + [
            Node(
                package="cobot_magic_cameras",
                executable="opencv_rgb_cameras",
                name="cobot_magic_rgb_cameras",
                output="screen",
                parameters=[
                    {
                        "width": LaunchConfiguration("width"),
                        "height": LaunchConfiguration("height"),
                        "fps": LaunchConfiguration("fps"),
                        "fourcc": LaunchConfiguration("fourcc"),
                        "camera_f_device": LaunchConfiguration("camera_f_device"),
                        "camera_l_device": LaunchConfiguration("camera_l_device"),
                        "camera_r_device": LaunchConfiguration("camera_r_device"),
                        "camera_f_serial": LaunchConfiguration("camera_f_serial"),
                        "camera_l_serial": LaunchConfiguration("camera_l_serial"),
                        "camera_r_serial": LaunchConfiguration("camera_r_serial"),
                    }
                ],
            ),
        ]
    )
