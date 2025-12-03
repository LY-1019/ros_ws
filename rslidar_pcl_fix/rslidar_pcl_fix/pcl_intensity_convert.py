#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class IntensityFixNode(Node):
    def __init__(self):
        super().__init__('pcl_intensity_fix')

        self.sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.cb,
            qos_profile=qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/rslidar_points_fixed',
            10
        )

    def cb(self, msg):
        points = pc2.read_points_list(
            msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True
        )

        fixed_points = [
            [float(x), float(y), float(z), float(i)] for (x, y, z, i) in points
        ]

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        new_msg = pc2.create_cloud(msg.header, fields, fixed_points)
        new_msg.header = msg.header

        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IntensityFixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
