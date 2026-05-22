from glob import glob
import os

from setuptools import setup

package_name = "px4_calibration"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Duckietown",
    maintainer_email="info@duckietown.com",
    description="PX4 calibration helpers for DD24 through MAVROS2 and MAVLink.",
    license="Duckietown License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "px4_calibration_node = px4_calibration.px4_calibration_node:main",
            "px4_manual_calibration = px4_calibration.manual_calibration:main",
        ],
    },
)
