from setuptools import setup
import os
from glob import glob

package_name = 'slam_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ly',
    maintainer_email='your@email.com',
    description='Fusion SLAM for LiDAR + RGBD',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
