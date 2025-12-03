from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # 小车底盘坐标为 base_link

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_lidar",
            arguments=[
                "0.15", "0.0", "0.20", "0", "0", "0",
                "base_link", "rslidar"
            ]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_camera",
            arguments=[
                "0.10", "0.05", "0.18", "-1.57", "0", "-1.57",
                "base_link", "camera_link"
            ]
        )
    ])
