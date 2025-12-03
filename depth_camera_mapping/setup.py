from setuptools import find_packages, setup

package_name = 'depth_camera_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ('share/' + package_name + '/launch', ['launch/robot_rtabmap.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abd',
    maintainer_email='abd@todo.todo',
    description='A ROS 2 package for depth camera mapping using SLAM',
    license='Apache-2.0',  # 请确保这里是您包使用的许可证
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 如果有其他可执行节点可以在这里添加
        ],
    },
)
