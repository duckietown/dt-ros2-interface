from setuptools import setup
import os
from glob import glob

package_name = 'image_relayer'

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
 description='ROS2 to DTPS image relayer',
 license='None',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
        "image_relayer_node = image_relayer.image_relayer_node:main"
     ],
   },
)
