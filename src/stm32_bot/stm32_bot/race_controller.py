#!/usr/bin/env python3
"""
Contrôleur de course Pure Pursuit.
- Utilise le frame MAP (absolu via AMCL) — stable entre les redémarrages
- Attend que l'utilisateur appuie sur Entrée avant de démarrer
- Suit la trajectoire en boucle fermée
- Dépassement : se décale vers le côté le plus large quand obstacle détecté
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import tf2_ros
import yaml
import os
import math


class RaceController(Node):
    def __init__(self):
        super().__init__('race_controller')

        self.declare_parameter('path_file',             '/tmp/track.yaml')
        self.declare_parameter('lookahead_dist',        0.6)
        self.declare_parameter('max_speed',             2.0)
        self.declare_parameter('min_speed',             0.3)
        self.declare_parameter('speed_curvature_gain',  1.0)
        self.declare_parameter('obstacle_threshold',    1.5)
        self.declare_parameter('emergency_stop_dist',   0.15)
        self.declare_parameter('lateral_offset',        0.3)
        self.declare_parameter('overtake_speed_boost',  0.2)

        self.ld            = self.get_parameter('lookahead_dist').value
        self.max_speed     = self.get_parameter('max_speed').value
        self.min_speed     = self.get_parameter('min_speed').value
        self.curve_gain    = self.get_parameter('speed_curvature_gain').value
        self.obs_thr       = self.get_parameter('obstacle_threshold').value
        self.stop_dist     = self.get_parameter('emergency_stop_dist').value
        self.lat_off       = self.get_parameter('lateral_offset').value
        self.speed_boost   = self.get_parameter('overtake_speed_boost').value
        path_file        = self.get_parameter('path_file').value

        self.path = self._load_path(path_file)
        self.n    = len(self.path)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nearest_idx = 0
        self.finished    = False
        self.state       = 'RACING'
        self.scan        = None
        self.started     = False   # ← attend le service /race/start

        self.pub_cmd       = self.create_publisher(Twist,        '/cmd_vel',         10)
        self.pub_path      = self.create_publisher(Path,         '/race/path',        1)
        self.pub_lookahead = self.create_publisher(PointStamped, '/race/lookahead',  10)

        self.create_subscription(LaserScan, '/scan_filtered', self.scan_cb, 10)
        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0,  self.publish_path)

        self.create_service(Empty, '/race/start', self._start_cb)

        self.get_logger().info('=== RACE CONTROLLER prêt (frame: map) ===')
        self.get_logger().info(
            f'{self.n} points | vitesse {self.min_speed}-{self.max_speed} m/s | lookahead {self.ld}m')
        self.get_logger().info(
            f'Obstacle <{self.obs_thr}m → dépassement | <{self.stop_dist}m → arrêt urgence')
        self.get_logger().info('Dans RViz2 : Add → Path → /race/path  |  Add → PointStamped → /race/lookahead')
        self.get_logger().info('')
        self.get_logger().info('>>> Fais le "2D Pose Estimate" dans RViz2, puis dans un autre terminal :')
        self.get_logger().info('    ros2 service call /race/start std_srvs/srv/Empty "{}"')

    # ════════════════════════════════════════════════════
    def _start_cb(self, request, response):
        self.started = True
        # Trouver le point de départ le plus proche de la position actuelle
        pose = self._get_pose()
        if pose is not None:
            rx, ry, _ = pose
            best_d, best_i = float('inf'), 0
            for i, (px, py) in enumerate(self.path):
                d = math.hypot(px - rx, py - ry)
                if d < best_d:
                    best_d, best_i = d, i
            self.nearest_idx = best_i
            self.get_logger().info(f'★ DÉPART depuis le point {best_i}/{self.n} (à {best_d:.2f}m)')
        else:
            self.get_logger().info('★ DÉPART (TF indisponible, départ depuis point 0)')
        return response

    # ════════════════════════════════════════════════════
    def _load_path(self, path_file):
        if not os.path.exists(path_file):
            self.get_logger().error(f'Fichier introuvable : {path_file}')
            raise FileNotFoundError(path_file)
        with open(path_file, 'r') as f:
            data = yaml.safe_load(f)
        pts = [(p['x'], p['y']) for p in data['path']]
        self.get_logger().info(f'Trajectoire chargée : {len(pts)} points')
        return pts

    # ════════════════════════════════════════════════════
    def scan_cb(self, msg):
        self.scan = msg

    def _get_pose(self):
        try:
            t   = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x   = t.transform.translation.x
            y   = t.transform.translation.y
            q   = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return x, y, yaw
        except Exception:
            return None

    # ════════════════════════════════════════════════════
    def _find_lookahead(self, rx, ry, lateral_shift=0.0):
        # Point le plus proche — cherche en avant uniquement
        best_d = float('inf')
        best_i = self.nearest_idx
        for i in range(60):
            idx = (self.nearest_idx + i) % self.n
            d = math.hypot(self.path[idx][0] - rx, self.path[idx][1] - ry)
            if d < best_d:
                best_d, best_i = d, idx
            elif i > 5 and d > best_d * 1.5:
                break
        self.nearest_idx = best_i

        # Avancer jusqu'à la distance ld (boucle)
        idx = self.nearest_idx
        for _ in range(self.n):
            if math.hypot(self.path[idx][0] - rx, self.path[idx][1] - ry) >= self.ld:
                break
            idx = (idx + 1) % self.n

        lx, ly = self.path[idx]

        # Décalage latéral (dépassement)
        if abs(lateral_shift) > 0.01:
            ni  = (idx + 1) % self.n
            ang = math.atan2(self.path[ni][1] - ly, self.path[ni][0] - lx)
            perp = ang + math.pi / 2.0
            lx += lateral_shift * math.cos(perp)
            ly += lateral_shift * math.sin(perp)

        return lx, ly

    # ════════════════════════════════════════════════════
    def _analyze_scan(self):
        if self.scan is None:
            return float('inf'), float('inf'), float('inf')
        ranges = self.scan.ranges
        amin   = self.scan.angle_min
        ainc   = self.scan.angle_increment
        rmin   = self.scan.range_min
        rmax   = self.scan.range_max
        n      = len(ranges)

        def valid(r):
            return rmin < r < rmax and math.isfinite(r)

        # Arc avant ±25° autour de π/2
        fc    = int((math.pi / 2.0 - amin) / ainc)
        sp    = max(1, int(math.radians(25) / ainc))
        fvals = [ranges[i] for i in range(max(0, fc-sp), min(n, fc+sp)) if valid(ranges[i])]
        min_front = min(fvals) if fvals else float('inf')

        # Côtés (15° depuis chaque bord)
        ss    = max(1, int(math.radians(15) / ainc))
        rvals = [ranges[i] for i in range(0,            min(n, ss))       if valid(ranges[i])]
        lvals = [ranges[i] for i in range(max(0, n-ss), n)               if valid(ranges[i])]
        d_r   = sum(rvals)/len(rvals) if rvals else float('inf')
        d_l   = sum(lvals)/len(lvals) if lvals else float('inf')
        return min_front, d_l, d_r

    # ════════════════════════════════════════════════════
    def control_loop(self):
        if not self.started:
            return


        pose = self._get_pose()
        if pose is None:
            self.get_logger().warn('En attente de la TF map→base_link...', throttle_duration_sec=2.0)
            return
        rx, ry, ryaw = pose

        min_front, _, _ = self._analyze_scan()
        if min_front < self.stop_dist:
            self.pub_cmd.publish(Twist())
            self.get_logger().warn(
                f'ARRÊT URGENCE — obstacle à {min_front:.2f}m', throttle_duration_sec=1.0)
            return

        # Lookahead
        lx, ly = self._find_lookahead(rx, ry, 0.0)

        pt = PointStamped()
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.header.frame_id = 'map'
        pt.point.x = lx
        pt.point.y = ly
        self.pub_lookahead.publish(pt)

        # Pure Pursuit — lookahead en frame robot
        dx   =  (lx - rx) * math.cos(-ryaw) - (ly - ry) * math.sin(-ryaw)
        dy   =  (lx - rx) * math.sin(-ryaw) + (ly - ry) * math.cos(-ryaw)
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            return

        curvature = 2.0 * dy / (dist * dist)

        # Vitesse adaptative : ralentir dans les virages serrés
        speed = self.max_speed / (1.0 + self.curve_gain * abs(curvature))
        speed = max(self.min_speed, speed)

        cmd = Twist()
        cmd.linear.x  = speed
        cmd.angular.z = speed * curvature
        self.pub_cmd.publish(cmd)

    # ════════════════════════════════════════════════════
    def publish_path(self):
        now = self.get_clock().now().to_msg()
        msg = Path()
        msg.header.stamp    = now
        msg.header.frame_id = 'map'
        for x, y in self.path:
            ps = PoseStamped()
            ps.header.stamp    = now
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RaceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
