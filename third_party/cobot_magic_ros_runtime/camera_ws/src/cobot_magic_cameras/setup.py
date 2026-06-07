from setuptools import find_packages, setup

package_name = "cobot_magic_cameras"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/cobot_magic_rgb.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arx",
    maintainer_email="arx@todo.todo",
    description="ROS2 OpenCV RGB camera publishers for Cobot Magic.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "opencv_rgb_cameras = cobot_magic_cameras.opencv_rgb_cameras:main",
        ],
    },
)
