#!/usr/bin/env python3
"""
Nœud de deskewing lidar.

Pour chaque scan, interpole la pose du robot entre le début et la fin
de l'acquisition (via TF2 odom→base_laser), ramène tous les points dans
le référentiel de fin de scan. Applique aussi le filtre 180° avant.

Publie /scan_deskewed à la place de /scan_filtered pour SLAM Toolbox.
Le /scan_filtered existant reste actif pour la détection d'obstacles.
"""
import math
import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros

KEEP_MIN = 0.0
KEEP_MAX = math.pi


class LidarDeskewer(Node):
    def __init__(self):
        super().__init__('lidar_deskewer')

        # Buffer TF suffisamment grand pour stocker les poses passées
        self.tf_buffer   = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_deskewed', 10)

        self._warn_count = 0
        self.get_logger().info('Lidar deskewer démarré — publie /scan_deskewed')

    # ──────────────────────────────────────────────────────
    def _yaw_from_tf(self, tf):
        q = tf.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ──────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        # Deskewing désactivé pour test — filtre 180° + offset timestamp uniquement
        self._publish_filtered_only(msg)

    # ──────────────────────────────────────────────────────
    def _publish_filtered_only(self, msg):
        """Fallback : filtre 180° + offset -50ms sans deskewing."""
        n      = len(msg.ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        ranges = np.where((angles >= KEEP_MIN) & (angles <= KEEP_MAX), ranges, np.inf)

        out = LaserScan()
        out.header          = msg.header
        t_offset = (rclpy.time.Time.from_msg(msg.header.stamp)
                    - rclpy.duration.Duration(nanoseconds=50_000_000))
        out.header.stamp    = t_offset.to_msg()
        out.angle_min       = msg.angle_min
        out.angle_max       = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment  = msg.time_increment
        out.scan_time       = msg.scan_time
        out.range_min       = msg.range_min
        out.range_max       = msg.range_max
        out.ranges          = ranges.tolist()
        out.intensities     = msg.intensities
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDeskewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()