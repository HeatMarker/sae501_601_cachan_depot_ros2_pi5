#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

# 180° avant du robot (en frame laser, avec TF yaw=-1.57 sur base_laser)
# Ajuste si les mauvais points sont filtrés : essaie -3.14159 / 0.0
KEEP_MIN = 0.0
KEEP_MAX = math.pi

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.create_subscription(LaserScan, '/scan', self.callback, 10)

    def callback(self, msg):
        ranges = list(msg.ranges)
        for i in range(len(ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            if angle < KEEP_MIN or angle > KEEP_MAX:
                ranges[i] = float('inf')

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = list(msg.intensities) if msg.intensities else []
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
