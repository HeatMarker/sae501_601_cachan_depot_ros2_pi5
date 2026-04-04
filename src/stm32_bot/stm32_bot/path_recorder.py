#!/usr/bin/env python3
"""
Enregistre la trajectoire pendant la téléopération.
- Utilise le frame MAP (absolu via AMCL) — stable entre les redémarrages
- Publie /race/recorded_path (nav_msgs/Path) → visible dans RViz2 (add Path display)
- Détecte automatiquement la fermeture de boucle
- Sauvegarde sur Ctrl+C ou auto-sauvegarde à la fermeture de boucle
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import yaml
import os
import math
import signal


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        self.declare_parameter('output_file',     '/tmp/track.yaml')
        self.declare_parameter('min_dist',         0.15)
        self.declare_parameter('loop_close_dist',  0.5)
        self.declare_parameter('loop_min_points',  30)

        self.output_file  = self.get_parameter('output_file').value
        self.min_dist     = self.get_parameter('min_dist').value
        self.loop_dist    = self.get_parameter('loop_close_dist').value
        self.loop_min_pts = self.get_parameter('loop_min_points').value

        self.poses       = []
        self.last_x      = None
        self.last_y      = None
        self.loop_closed = False

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Path, '/race/recorded_path', 1)

        # Enregistrement à 5 Hz
        self.create_timer(0.2, self.record_tick)
        # Publication RViz2 à 2 Hz
        self.create_timer(0.5, self.publish_path)

        signal.signal(signal.SIGINT, self._on_sigint)

        self.get_logger().info('=== PATH RECORDER démarré (frame: map) ===')
        self.get_logger().info(f'Sortie : {self.output_file}')
        self.get_logger().info('Dans RViz2 : Add → Path → topic /race/recorded_path')
        self.get_logger().info('Conduire un tour → sauvegarde auto à la fermeture de boucle (ou Ctrl+C)')

    # ──────────────────────────────────────────────────
    def record_tick(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation

        if self.last_x is None:
            self._add(x, y, q)
            return

        if math.hypot(x - self.last_x, y - self.last_y) < self.min_dist:
            return

        self._add(x, y, q)

        # Détection fermeture de boucle
        if not self.loop_closed and len(self.poses) >= self.loop_min_pts:
            p0 = self.poses[0]
            if math.hypot(x - p0['x'], y - p0['y']) < self.loop_dist:
                self.loop_closed = True
                self.get_logger().info(
                    f'BOUCLE FERMÉE ({len(self.poses)} points) → sauvegarde automatique')
                self.save()

    def _add(self, x, y, q):
        self.poses.append({
            'x':  float(x), 'y':  float(y),
            'qz': float(q.z), 'qw': float(q.w),
        })
        self.last_x = x
        self.last_y = y
        n = len(self.poses)
        if n % 20 == 0:
            self.get_logger().info(f'{n} points enregistrés')

    # ──────────────────────────────────────────────────
    def publish_path(self):
        if not self.poses:
            return
        now  = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp    = now
        path.header.frame_id = 'map'
        for p in self.poses:
            ps = PoseStamped()
            ps.header.stamp    = now
            ps.header.frame_id = 'map'
            ps.pose.position.x = p['x']
            ps.pose.position.y = p['y']
            ps.pose.orientation.z = p['qz']
            ps.pose.orientation.w = p['qw']
            path.poses.append(ps)
        self.pub.publish(path)

    # ──────────────────────────────────────────────────
    def save(self):
        if len(self.poses) < 5:
            self.get_logger().warn('Moins de 5 points — rien à sauvegarder')
            return
        # Fermer la boucle
        self.poses.append(self.poses[0].copy())

        os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)
        with open(self.output_file, 'w') as f:
            yaml.dump({'frame': 'map', 'path': self.poses}, f, default_flow_style=False)
        self.get_logger().info(
            f'✓ Sauvegardé : {len(self.poses)} points → {self.output_file}')

    def _on_sigint(self, sig, frame):
        self.get_logger().info('Ctrl+C → sauvegarde...')
        self.save()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
