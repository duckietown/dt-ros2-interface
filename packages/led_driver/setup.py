from setuptools import setup
import os
from glob import glob

package_name = 'led_driver'

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
 maintainer='Akshet Patel',
 maintainer_email='akshet.patel@duckietown.com',
 description='ROS2 LED Driver Package',
 license='None',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
        "led_driver_node = led_driver.led_driver_node:main"
     ],
   },
)
