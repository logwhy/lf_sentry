import os

from setuptools import find_packages, setup

package_name = "teleop_gimbal_keyboard"
share_path = "share/" + package_name


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        (share_path, ["package.xml"]),
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Lihan Chen",
    author_email="lihanchen2004@163.com",
    maintainer="Lihan Chen",
    maintainer_email="lihanchen2004@163.com",
    url="",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache-2.0",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="A robot-agnostic teleoperation node to convert keyboard commands to JointState messages",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teleop_gimbal_keyboard = teleop_gimbal_keyboard.teleop_gimbal_keyboard:main",
        ],
    },
)
