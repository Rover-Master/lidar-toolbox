from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'lidar_toolbox'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adnana',
    maintainer_email='adnanabdullah@ufl.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        'go_to_point = lidar_toolbox.go_to_point:main',
        'follow_wall = lidar_toolbox.follow_wall:main',
        'bug0 = lidar_toolbox.bug0:main',
        'bug1 = lidar_toolbox.bug1:main',
        'laser_scan_subscriber = lidar_toolbox.laser_scan_subscriber:main',
        ],
    },
)
