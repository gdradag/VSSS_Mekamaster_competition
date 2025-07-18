from setuptools import setup
import os
from glob import glob

package_name = "robot_mqtt"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    install_requires=["setuptools", "paho-mqtt"],
    zip_safe=True,
    maintainer="tu_usuario",
    maintainer_email="tu_email@example.com",
    description="Nodo que comunica ROS 2 con ESP32 usando MQTT",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_mqtt = robot_mqtt.mqtt_publisher:main",
            "teleop_interface = robot_mqtt.teleop_interface:main",
            "teleop_node = robot_mqtt.teleop_node:main",
            "teleop_Snode = robot_mqtt.teleop_Snode:main",
            "teleop_interface2 = robot_mqtt.teleop_interface2:main",
            "teleop_node2 = robot_mqtt.teleop_node2:main",
            "teleop_Snode2 = robot_mqtt.teleop_Snode2:main",
            "velocity_calculator = robot_mqtt.velocity_calculator:main",
            "video_publisher = robot_mqtt.video_publisher:main",
            "robot_tracker = robot_mqtt.robot_tracker:main"
            "position_estimator = robot_mqtt.position_estimator",

        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
)

