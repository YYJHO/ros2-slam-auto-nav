from glob import glob
import os

from setuptools import setup

package_name = "virtual_indoor_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "data"), glob("data/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="YYJHO",
    maintainer_email="317507360@qq.com",
    description="Virtual indoor navigation system with ROS 2 Humble and Gazebo Classic.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "room_nav_node = virtual_indoor_nav.room_nav_node:main",
            "wsad_teleop = virtual_indoor_nav.wsad_teleop:main",
            "control_center = virtual_indoor_nav.control_center:main",
            "auto_explorer = virtual_indoor_nav.auto_explorer:main",
        ],
    },
)
