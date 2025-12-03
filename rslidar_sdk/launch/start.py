from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    config_path = "/home/ly/ros2_ws/src/rslidar_sdk/config/config.yaml"
    
    return LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),

        # ---------------------------
        # 1. RoboSense LiDAR SDK 节点
        # ---------------------------
        Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name='rslidar_sdk_node',
            output='screen',
            arguments=['--config', config_path]
        ),

        # ---------------------------
        # 2. 添加 TF 静态变换（base_link -> rslidar）
        # ---------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rslidar_tf',
            arguments=[
                "0.0",  # x
                "0.0",  # y
                "0.0",  # z
                "0.0",  # roll (rad)
                "0.0",  # pitch (rad)
                "0.0",  # yaw (rad)
                "base_link",  # 父坐标系
                "rslidar"     # 子坐标系（你的点云 frame_id）
            ]
        ),

        # ---------------------------
        # 3. RViz2 可视化
        # ---------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', "/home/ly/ros2_ws/src/rslidar_sdk/config/rslidar.rviz"]
        )
    ])
