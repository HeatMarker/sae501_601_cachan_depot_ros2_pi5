#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ImuVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.get_logger().info("IMU Visualizer démarré — en écoute sur /imu_data")

    def imu_callback(self, msg):
        q = msg.orientation
        # Si le driver ne publie pas d'orientation, on l'ignore
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            self.get_logger().warn("Orientation nulle dans /imu_data — le driver ne publie pas de quaternion ?", throttle_duration_sec=5.0)
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu_visual'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
