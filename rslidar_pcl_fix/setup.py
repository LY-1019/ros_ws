from setuptools import setup

package_name = 'rslidar_pcl_fix'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_data={package_name: ['*.py']},   # ★★★ 必须添加这一行！
    include_package_data=True,               # ★★★ 必须添加这一行！
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@example.com',
    description='Fix intensity from uint8 to float32',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pcl_intensity_fix = rslidar_pcl_fix.pcl_intensity_convert:main',
        ],
    },
)
