from glob import glob
import os
from setuptools import find_packages, setup

package_name = "aquaculture_autonomous_robot"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "firmware"), glob("firmware/*.ino")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Senior Robotics & Embedded Systems Architect",
    maintainer_email="robotics@aquaculture.ai",
    description="Autonomous Aquaculture Feeding & Water-Quality Inspection Robot for ROS 2 Humble",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "esp32_sim_node = scripts.esp32_sim_node:main",
            "fish_vision_node = scripts.fish_vision_node:main",
            "autonomous_brain_node = scripts.autonomous_brain_node:main",
            "esp32_serial_bridge_node = scripts.esp32_serial_bridge_node:main",
        ],
    },
)
