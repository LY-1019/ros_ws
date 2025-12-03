from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    config_path = "/home/ly/ros2_ws/src/rslidar_sdk/config/config.yaml"
    
    return LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),

        # ----------------------------------------------------
        # 1. RoboSense LiDAR SDK 节点（发布 /rslidar_points）
        # ----------------------------------------------------
        Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name='rslidar_sdk_node',
            output='screen',
            arguments=['--config', config_path]
        ),

        # ----------------------------------------------------
        # 2. TF（base_link -> rslidar）
        # ----------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rslidar_tf',
            arguments=[
                "0", "0", "0",   # x y z
                "0", "0", "0",   # roll pitch yaw
                "base_link",
                "rslidar"
            ]
        ),

        # ----------------------------------------------------
        # 3. RTAB-Map 点云里程计（icp_odometry）
        # ----------------------------------------------------
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            remappings=[
                ("cloud", "/rslidar_points")
            ],
            parameters=[
                {"frame_id": "base_link"},
                {"approx_sync": False}
            ]
        ),

        # ----------------------------------------------------
        # 4. RTAB-Map 主节点（SLAM / 地图构建）
        # ----------------------------------------------------
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            remappings=[
                ("scan_cloud", "/rslidar_points"),
                ("odom", "/rtabmap/odom")
            ],
            parameters=[
                {"subscribe_scan_cloud": True},
                {"frame_id": "base_link"}
            ]
        ),

        # ----------------------------------------------------
        # 5. RViz2 可视化
        # ----------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', "/home/ly/ros2_ws/src/rslidar_sdk/config/rslidar.rviz"]
        )
    ])
