from setuptools import setup
import os
from glob import glob

package_name = 'rpi_camera_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Davide Iafrate',
    maintainer_email='davide.iafrate@duckietown.com',
    description='ROS 2 driver for the Raspberry Pi CSI camera (libcamera / picamera2)',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rpi_camera_driver_node = rpi_camera_driver.rpi_camera_driver_node:main",
        ],
    },
)
