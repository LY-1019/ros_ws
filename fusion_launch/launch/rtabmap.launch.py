from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # RGB-D 里程计节点（必须先跑 odom）
        Node(
            package='rtabmap_ros',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'approx_sync': True
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw')
            ]
        ),

        # RTAB-Map 主节点（建图）
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_scan_cloud': True,   # 启用 LiDAR
                'queue_size': 30,
                'approx_sync': True
            }],
            remappings=[
                ('scan_cloud', '/rslidar_points'),
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('odom', '/rtabmap/odom')
            ]
        ),

        # RTAB-Map 可视化界面
        Node(
            package='rtabmap_ros',
            executable='rtabmapviz',
            name='rtabmapviz',
            output='screen',
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('scan_cloud', '/rslidar_points'),
                ('odom', '/rtabmap/odom')
            ]
        )
    ])
