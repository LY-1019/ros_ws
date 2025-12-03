from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rslidar_tf",
            arguments=[
                "0.0",  # x
                "0.0",  # y
                "0.0",  # z
                "0.0",  # roll
                "0.0",  # pitch
                "0.0",  # yaw
                "base_link",  # 父坐标系
                "rslidar"     # 子坐标系 (你的点云 frame_id)
            ]
        )
    ])
