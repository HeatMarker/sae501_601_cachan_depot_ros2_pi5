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
        n  = len(msg.ranges)
        dt = msg.time_increment

        # LD06 peut envoyer time_increment=0 → on le calcule
        if dt < 1e-9:
            dt = msg.scan_time / n if msg.scan_time > 1e-9 else 0.0

        t_start = rclpy.time.Time.from_msg(msg.header.stamp)
        t_end   = (t_start + rclpy.duration.Duration(nanoseconds=int((n - 1) * dt * 1e9))
                   if dt > 0 else t_start)

        frame = msg.header.frame_id  # base_laser

        try:
            tf_s = self.tf_buffer.lookup_transform(
                'odom', frame, t_start,
                timeout=rclpy.duration.Duration(milliseconds=40))
            tf_e = self.tf_buffer.lookup_transform(
                'odom', frame, t_end,
                timeout=rclpy.duration.Duration(milliseconds=40))
        except Exception:
            # TF pas encore disponible au démarrage → fallback sans deskewing
            self._warn_count += 1
            if self._warn_count <= 5:
                self.get_logger().warn('TF indispo — scan republié sans deskewing')
            self._publish_filtered_only(msg)
            return

        self._warn_count = 0

        # ── Poses start / end ──────────────────────────────
        tx_s = tf_s.transform.translation.x
        ty_s = tf_s.transform.translation.y
        yaw_s = self._yaw_from_tf(tf_s)

        tx_e = tf_e.transform.translation.x
        ty_e = tf_e.transform.translation.y
        yaw_e = self._yaw_from_tf(tf_e)

        # ── Interpolation linéaire pour chaque point ───────
        idx    = np.arange(n, dtype=np.float64)
        alpha  = idx / max(n - 1, 1)          # 0 → 1

        tx_i   = tx_s  + alpha * (tx_e  - tx_s)
        ty_i   = ty_s  + alpha * (ty_e  - ty_s)
        yaw_i  = yaw_s + alpha * (yaw_e - yaw_s)  # ok pour petits angles (<0.3 rad)

        # ── Points en coordonnées polaires → cartésien (frame laser à t_i) ──
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + idx * msg.angle_increment
        valid  = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)

        px = np.where(valid, ranges * np.cos(angles), np.nan)
        py = np.where(valid, ranges * np.sin(angles), np.nan)

        # ── Vers odom à t_i ────────────────────────────────
        cos_i  = np.cos(yaw_i)
        sin_i  = np.sin(yaw_i)
        px_odom = tx_i + px * cos_i - py * sin_i
        py_odom = ty_i + px * sin_i + py * cos_i

        # ── Vers frame laser à t_end ───────────────────────
        cos_e = math.cos(-yaw_e)
        sin_e = math.sin(-yaw_e)
        dx = px_odom - tx_e
        dy = py_odom - ty_e
        px_desk = dx * cos_e - dy * sin_e
        py_desk = dx * sin_e + dy * cos_e

        new_ranges = np.where(valid, np.hypot(px_desk, py_desk), np.inf)

        # ── Filtre 180° avant ──────────────────────────────
        keep = (angles >= KEEP_MIN) & (angles <= KEEP_MAX)
        new_ranges = np.where(keep, new_ranges, np.inf)

        # ── Publication ────────────────────────────────────
        out = LaserScan()
        out.header           = msg.header
        out.header.stamp     = t_end.to_msg()   # timestamp = fin de scan
        out.angle_min        = msg.angle_min
        out.angle_max        = msg.angle_max
        out.angle_increment  = msg.angle_increment
        out.time_increment   = 0.0              # tous les points au même instant
        out.scan_time        = msg.scan_time
        out.range_min        = msg.range_min
        out.range_max        = msg.range_max
        out.ranges           = new_ranges.tolist()
        out.intensities      = msg.intensities

        self.pub.publish(out)

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