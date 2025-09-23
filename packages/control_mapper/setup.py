from setuptools import setup
import os
from glob import glob

package_name = 'control_mapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duckietown',
    maintainer_email='info@duckietown.com',
    description='ROS2 Control Mapper Package',
    license='Duckietown License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_mapper_node = control_mapper.control_mapper_node:main',
        ],
    },
)

