from setuptools import setup
import os
from glob import glob


package_name = 'duckiematrix_interface'


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Liam McAlpine',
    maintainer_email='liam.mcalpine@duckietown.com',
    description='ROS 2 bridge for the Duckiematrix state topic exposed over DTPS',
    license='Duckietown License',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'duckiematrix_interface_node = duckiematrix_interface.duckiematrix_interface_node:main',
        ],
    },
)