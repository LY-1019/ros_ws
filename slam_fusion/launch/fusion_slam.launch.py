from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('slam_fusion')

    # 1. 激光雷达驱动
    rslidar_sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rslidar_sdk'),
                'launch',
                'start.py'
            )
        )
    )

    # 2. Gemini 336L 驱动
    gemini_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('orbbec_camera'),
                'launch',
                'gemini_330_series.launch.py'
            )
        ),
        launch_arguments={
            'enable_colored_point_cloud': 'true'
        }.items()
    )

    # 3. TF 外参
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'config', 'robot_tf.launch.py')
        )
    )

    # 4. Cartographer SLAM（3D）
    carto_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_3d',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        arguments=[
            '-configuration_directory', os.path.join(pkg_dir, 'config'),
            '-configuration_basename', 'helios_3d.lua'
        ]
    )

   

    # 5. ORB-SLAM3 RGB-D
  

    # 6. robot_localization EKF
   

    return LaunchDescription([
        rslidar_sdk_launch,
        gemini_launch,
        tf_launch,
        carto_node,
        #orb_node,
        #ekf_node
    ])
