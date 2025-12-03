#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class DriftPrinter(Node):
    def __init__(self):
        super().__init__('drift_printer')

        self.start_pose = None
        self.end_pose = None

        self.sub = self.create_subscription(
            PoseStamped,
            '/current_pose',   # 你的 SLAM 位姿话题
            self.cb,
            10
        )

        print("DriftPrinter: Listening to /current_pose ...")

    def cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        if self.start_pose is None:
            self.start_pose = (x, y, z)
            print(f"[Start Pose] x={x:.3f}, y={y:.3f}, z={z:.3f}")

        self.end_pose = (x, y, z)

    def print_result(self):
        if self.start_pose is None or self.end_pose is None:
            print("No pose received.")
            return
        
        sx, sy, sz = self.start_pose
        ex, ey, ez = self.end_pose

        error = math.sqrt((ex-sx)**2 + (ey-sy)**2 + (ez-sz)**2)

        print("\n========= SLAM Drift =========")
        print(f"Start Pose: ({sx:.3f}, {sy:.3f}, {sz:.3f})")
        print(f"End Pose:   ({ex:.3f}, {ey:.3f}, {ez:.3f})")
        print(f"Total Drift Error: {error:.3f} m")
        print("================================\n")

def main(args=None):
    rclpy.init(args=args)
    node = DriftPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_result()   # ← 退出时打印误差
        node.destroy_node()    # 只销毁节点，不再调用 rclpy.shutdown()

if __name__ == '__main__':
    main()
