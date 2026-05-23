#!/usr/bin/env python3
"""
Blind Navigation — Follow the Gap
Navigation réactive sans carte ni odométrie.
Uniquement LIDAR + IMU.

Algorithme :
  1. Safety bubble autour de l'obstacle le plus proche
  2. Trouver le plus grand gap libre devant
  3. Cibler le point le plus loin dans ce gap
  4. P sur l'angle + D sur le yaw rate IMU pour amortir
  5. Vitesse proportionnelle à la distance devant
"""
import math
import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu


# angle=π/2 = droit devant dans /scan_filtered (TF lidar yaw=-1.57)
FRONT_ANGLE = math.pi / 2.0


class BlindNav(Node):
    def __init__(self):
        super().__init__('blind_nav')

        # ── Paramètres ────────────────────────────────────────────────────
        self.declare_parameter('max_speed',            0.8)   # m/s en ligne droite
        self.declare_parameter('min_speed',            0.3)   # m/s en virage serré
        self.declare_parameter('emergency_stop_dist',  0.15)  # m — arrêt d'urgence (voiture 18cm large)
        self.declare_parameter('safety_bubble_radius', 0.15)  # m — rayon bulle (légèrement > demi-largeur 9cm)
        self.declare_parameter('gap_threshold',        0.5)   # m — distance min pour un "gap" (piste 60cm)
        self.declare_parameter('steering_gain',        1.0)   # Kp angle error → angular.z
        self.declare_parameter('imu_d_gain',           0.0)   # Kd yaw rate → amortissement (0=désactivé)
        self.declare_parameter('front_arc_deg',        20.0)  # demi-arc avant pour arrêt urgence (étroit)
        self.declare_parameter('cone_half_deg',        70.0)  # demi-cône de recherche (±70° depuis l'avant)
        self.declare_parameter('dist_full_speed',      2.0)   # m — distance à partir de laquelle vitesse max
        self.declare_parameter('speed_curve_gain',    2.0)   # ratio freinage en virage (0=aucun, 5=fort)

        self.max_speed      = self.get_parameter('max_speed').value
        self.min_speed      = self.get_parameter('min_speed').value
        self.stop_dist      = self.get_parameter('emergency_stop_dist').value
        self.bubble_r       = self.get_parameter('safety_bubble_radius').value
        self.gap_thr        = self.get_parameter('gap_threshold').value
        self.Kp             = self.get_parameter('steering_gain').value
        self.Kd             = self.get_parameter('imu_d_gain').value
        self.front_arc      = math.radians(self.get_parameter('front_arc_deg').value)
        self.dist_full_speed = self.get_parameter('dist_full_speed').value
        self.curve_gain     = self.get_parameter('speed_curve_gain').value
        self.cone_half      = math.radians(self.get_parameter('cone_half_deg').value)

        self.scan      = None
        self.yaw_rate  = 0.0
        self.emergency = False

        # ── Publishers / Subscribers ──────────────────────────────────────
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan_filtered', self._scan_cb, 10)
        self.create_subscription(Imu, '/imu_data', self._imu_cb, 10)
        self.create_timer(0.05, self._control_loop)  # 20 Hz

        threading.Thread(target=self._keyboard_thread, daemon=True).start()

        self.get_logger().info('=== BLIND NAV démarré — Follow the Gap ===')
        self.get_logger().info('[ ESPACE ] = ARU — relancer pour reprendre')
        self.get_logger().info(
            f'v={self.min_speed}-{self.max_speed} m/s | stop<{self.stop_dist}m | '
            f'gap>{self.gap_thr}m | Kp={self.Kp} Kd={self.Kd}'
        )

    # ─────────────────────────────────────────────────────────────────────
    def _keyboard_thread(self):
        try:
            tty_f = open('/dev/tty', 'rb', buffering=0)
        except Exception as e:
            self.get_logger().warn(f'ARU clavier indisponible : {e}')
            return
        fd  = tty_f.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok() and not self.emergency:
                ch = tty_f.read(1)
                if ch == b' ':
                    self.emergency = True
                    self.pub_cmd.publish(Twist())
                    self.get_logger().error('!!! ARU — relancer pour reprendre !!!')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            tty_f.close()

    # ─────────────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        self.scan = msg

    def _imu_cb(self, msg: Imu):
        # IMU physique : X=haut(robot_Z), Y=gauche(robot_Y), Z=arrière(-robot_X)
        # Yaw robot (rotation autour de robot_Z) = angular_velocity.x dans frame IMU brut
        self.yaw_rate = msg.angular_velocity.x

    # ─────────────────────────────────────────────────────────────────────
    def _control_loop(self):
        if self.emergency:
            self.pub_cmd.publish(Twist())
            return
        if self.scan is None:
            return

        msg  = self.scan
        n    = len(msg.ranges)
        amin = msg.angle_min
        ainc = msg.angle_increment
        rmin = msg.range_min
        rmax = msg.range_max

        # ── 1. Nettoyer les ranges ────────────────────────────────────────
        # inf/nan ou hors bornes → traité comme rmax (mur au fond)
        ranges = []
        for r in msg.ranges:
            if math.isfinite(r) and rmin < r < rmax:
                ranges.append(r)
            else:
                ranges.append(rmax)

        # ── 2. Distance min dans l'arc avant ─────────────────────────────
        fc      = int((FRONT_ANGLE - amin) / ainc)
        sp      = max(1, int(self.front_arc / ainc))
        fwd     = [ranges[i] for i in range(max(0, fc - sp), min(n, fc + sp))]
        min_front = min(fwd) if fwd else rmax

        # ── 3. Arrêt d'urgence ────────────────────────────────────────────
        if min_front < self.stop_dist:
            self.pub_cmd.publish(Twist())
            self.get_logger().warn(
                f'[STOP] obstacle à {min_front:.2f}m',
                throttle_duration_sec=0.5
            )
            return

        # ── 4. Safety bubble — masquer autour de l'obstacle le plus proche
        closest_idx = min(range(n), key=lambda i: ranges[i])
        closest_r   = ranges[closest_idx]
        if closest_r > 0.01:
            bubble_half = max(1, int(math.atan2(self.bubble_r, closest_r) / ainc))
            for i in range(max(0, closest_idx - bubble_half),
                           min(n, closest_idx + bubble_half + 1)):
                ranges[i] = 0.0

        # ── 5. Direction cible : moyenne pondérée dans le cône avant ────────
        # Poids = distance (points lointains = espace libre = fort poids).
        # En ligne droite : distribution symétrique → résultat = π/2 (tout droit).
        # En virage : côté ouvert plus loin → résultat décalé vers l'espace libre.
        # Robuste aux murs invisibles (rayons rasants) : s'ils ne sont pas détectés,
        # ils n'ajoutent pas de poids → la moyenne reste vers l'avant.
        c_start = max(0,   int((FRONT_ANGLE - self.cone_half - amin) / ainc))
        c_end   = min(n,   int((FRONT_ANGLE + self.cone_half - amin) / ainc) + 1)

        total_w        = 0.0
        weighted_angle = 0.0
        for i in range(c_start, c_end):
            w = max(0.0, ranges[i] - self.gap_thr)  # seul l'espace libre compte
            angle_i = amin + i * ainc
            weighted_angle += w * angle_i
            total_w        += w

        if total_w > 0.0:
            target_angle = weighted_angle / total_w
            gap_info = f'cone [{c_start}:{c_end}]'
        else:
            target_angle = FRONT_ANGLE  # fallback : tout droit
            gap_info = 'fallback'

        # ── 7. Commande de direction ──────────────────────────────────────
        heading_error = target_angle - FRONT_ANGLE  # positif = gap à gauche
        steering = self.Kp * heading_error - self.Kd * self.yaw_rate
        steering = max(-1.5, min(1.5, steering))

        # ── 8. Vitesse adaptative ─────────────────────────────────────────
        # Par obstacle : vitesse max à dist_full_speed, vitesse min à stop_dist
        obs_factor = (min_front - self.stop_dist) / max(self.dist_full_speed - self.stop_dist, 0.1)
        speed_obs  = self.min_speed + (self.max_speed - self.min_speed) * min(1.0, max(0.0, obs_factor))
        # Par virage : ralentir proportionnellement à l'angle de braquage
        speed_turn = self.max_speed / (1.0 + self.curve_gain * abs(steering))
        speed = max(self.min_speed, min(speed_obs, speed_turn))

        cmd = Twist()
        cmd.linear.x  = speed
        cmd.angular.z = steering
        self.pub_cmd.publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    node = BlindNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
