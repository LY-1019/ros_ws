from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    # 相机启动文件路径
    orbbec_launch_file = os.path.join(
        FindPackageShare('orbbec_camera').find('orbbec_camera'),
        'launch',
        'gemini_330_series.launch.py'
    )

    # 激光雷达配置路径
    lidar_config_path = "/home/ly/ros2_ws/src/rslidar_sdk/config/config.yaml"

    return LaunchDescription([

        # -----------------------------
        # 1. 启动 RS-Helios-16P 激光雷达
        # -----------------------------
        Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name='rslidar_sdk_node',
            output='screen',
            arguments=['--config', lidar_config_path]
        ),

        # TF: base_link -> rslidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                "0.0", "0.0", "0.0",    # x y z
                "0.0", "0.0", "0.0",    # roll pitch yaw
                "base_link", "rslidar"
            ]
        ),

        # -----------------------------
        # 2. 启动 Orbbec Gemini 336L 相机
        # -----------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch_file)
        ),

        # TF: base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf',
            arguments=[
                "0.10", "0.00", "0.15",  # 相机位置，需要你给真实值
                "0.0", "0.0", "0.0",     # 相机姿态
                "base_link", "camera_link"
            ]
        ),

        # -----------------------------
        # 3. 启动 RViz（展示相机 + 雷达融合）
        # -----------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
