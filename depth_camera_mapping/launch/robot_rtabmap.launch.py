import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    qos = LaunchConfiguration('qos', default='1')
    localization = LaunchConfiguration('localization', default='false')
    
    # RTAB-Map node
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',  # 配置框架
            'subscribe_depth': True,  # 订阅深度图像
            'subscribe_rgb': True,  # 订阅RGB图像
            'approx_sync': True,  # 同步图像和深度数据
            'queue_size': 100,
            'qos': qos,
            'use_sim_time': use_sim_time,
            # RTAB-Map参数
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
            'grid_map': 'true',  # 启用2D网格地图
            'Grid/FromDepth': 'true',  # 如果使用深度图像启用
            'Grid/MaxGroundHeight': '0.1',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/CellSize': '0.05',
            'Grid/RangeMax': '5.0',
            'Grid/3D': 'false',  # 强制2D模式
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),  # RGB 图像话题
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),  # 深度图像话题
            ('scan', '/scan'),  # 如果使用LiDAR，请使用这个话题
            ('odom', '/odometry/filtered'),
            ('grid_map', '/map'),  # 将地图映射到标准的 /map 话题
        ]
    )
    
    # 发布静态变换从 camera_link 到 map
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'map']  # 变换方向：从 camera_link 到 map
    )
    
    # RViz2 for visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',  # test auto push
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('depth_camera_mapping'), 'rviz', 'rtabmap.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('qos', default_value='1'),
        DeclareLaunchArgument('localization', default_value='false'),
        rtabmap,
        static_transform,  # 添加静态变换发布
        rviz2
    ])  

