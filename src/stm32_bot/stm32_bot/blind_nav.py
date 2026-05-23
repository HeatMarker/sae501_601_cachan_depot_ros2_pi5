#!/usr/bin/env python3
"""
Blind Navigation — Follow the Gap
Navigation réactive sans carte ni odométrie. Uniquement LIDAR + IMU.

Deux boucles :
  - 10 Hz (scan_cb)  : calcul complet depuis le scan lidar
  - 50 Hz (fast_loop): correction IMU entre les scans (delta_yaw depuis dernier scan)
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


FRONT_ANGLE = math.pi / 2.0  # angle=π/2 = droit devant dans /scan_filtered


class BlindNav(Node):
    def __init__(self):
        super().__init__('blind_nav')

        # ── Paramètres ────────────────────────────────────────────────────
        self.declare_parameter('max_speed',            0.8)
        self.declare_parameter('min_speed',            0.3)
        self.declare_parameter('emergency_stop_dist',  0.15)
        self.declare_parameter('safety_bubble_radius', 0.15)
        self.declare_parameter('gap_threshold',        0.5)
        self.declare_parameter('steering_gain',        2.0)
        self.declare_parameter('imu_d_gain',           0.0)
        self.declare_parameter('front_arc_deg',        20.0)
        self.declare_parameter('cone_half_deg',        70.0)
        self.declare_parameter('dist_full_speed',      2.0)
        self.declare_parameter('speed_curve_gain',     2.0)

        self.max_speed       = self.get_parameter('max_speed').value
        self.min_speed       = self.get_parameter('min_speed').value
        self.stop_dist       = self.get_parameter('emergency_stop_dist').value
        self.bubble_r        = self.get_parameter('safety_bubble_radius').value
        self.gap_thr         = self.get_parameter('gap_threshold').value
        self.Kp              = self.get_parameter('steering_gain').value
        self.Kd              = self.get_parameter('imu_d_gain').value
        self.front_arc       = math.radians(self.get_parameter('front_arc_deg').value)
        self.cone_half       = math.radians(self.get_parameter('cone_half_deg').value)
        self.dist_full_speed = self.get_parameter('dist_full_speed').value
        self.curve_gain      = self.get_parameter('speed_curve_gain').value

        # ── État partagé scan → fast_loop ────────────────────────────────
        self.last_target_angle = None   # cible dans frame robot au moment du scan
        self.last_speed        = 0.0
        self.emergency         = False

        # IMU : intégration yaw entre deux scans
        self.yaw_rate   = 0.0
        self.delta_yaw  = 0.0          # rotation accumulée depuis dernier scan
        self.last_imu_t = None

        # ── Publishers / Subscribers ──────────────────────────────────────
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan_filtered', self._scan_cb, 10)
        self.create_subscription(Imu, '/imu_data', self._imu_cb, 10)
        self.create_timer(0.02, self._fast_loop)   # 50 Hz — correction IMU

        threading.Thread(target=self._keyboard_thread, daemon=True).start()

        self.get_logger().info('=== BLIND NAV démarré (10 Hz scan + 50 Hz IMU) ===')
        self.get_logger().info('[ ESPACE ] = ARU — relancer pour reprendre')
        self.get_logger().info(
            f'v={self.min_speed}-{self.max_speed} m/s | stop<{self.stop_dist}m | Kp={self.Kp}'
        )

    # ── ARU clavier ───────────────────────────────────────────────────────
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

    # ── IMU : accumule la rotation depuis le dernier scan ─────────────────
    def _imu_cb(self, msg: Imu):
        # IMU physique : X=haut(robot_Z) → yaw robot = angular_velocity.x
        self.yaw_rate = msg.angular_velocity.x
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_imu_t is not None:
            dt = now - self.last_imu_t
            if 0.0 < dt < 0.1:
                self.delta_yaw += self.yaw_rate * dt
        self.last_imu_t = now

    # ── Scan 10 Hz : calcul complet ───────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        if self.emergency:
            return

        n    = len(msg.ranges)
        amin = msg.angle_min
        ainc = msg.angle_increment
        rmin = msg.range_min
        rmax = msg.range_max

        # 1. Nettoyer les ranges
        ranges = []
        for r in msg.ranges:
            if math.isfinite(r) and rmin < r < rmax:
                ranges.append(r)
            else:
                ranges.append(rmax)

        # 2. Distance min arc avant (arrêt urgence)
        fc        = int((FRONT_ANGLE - amin) / ainc)
        sp        = max(1, int(self.front_arc / ainc))
        fwd       = [ranges[i] for i in range(max(0, fc - sp), min(n, fc + sp))]
        min_front = min(fwd) if fwd else rmax

        if min_front < self.stop_dist:
            self.last_target_angle = None
            self.pub_cmd.publish(Twist())
            self.get_logger().warn(f'[STOP] {min_front:.2f}m', throttle_duration_sec=0.5)
            return

        # 3. Safety bubble
        closest_idx = min(range(n), key=lambda i: ranges[i])
        closest_r   = ranges[closest_idx]
        if closest_r > 0.01:
            bubble_half = max(1, int(math.atan2(self.bubble_r, closest_r) / ainc))
            for i in range(max(0, closest_idx - bubble_half),
                           min(n, closest_idx + bubble_half + 1)):
                ranges[i] = 0.0

        # 4. Moyenne pondérée dans le cône avant
        c_start = max(0, int((FRONT_ANGLE - self.cone_half - amin) / ainc))
        c_end   = min(n, int((FRONT_ANGLE + self.cone_half - amin) / ainc) + 1)

        total_w = 0.0
        w_angle = 0.0
        for i in range(c_start, c_end):
            w = max(0.0, ranges[i] - self.gap_thr)
            w_angle += w * (amin + i * ainc)
            total_w += w

        target_angle = (w_angle / total_w) if total_w > 0.0 else FRONT_ANGLE

        # 5. Vitesse
        obs_factor = (min_front - self.stop_dist) / max(self.dist_full_speed - self.stop_dist, 0.1)
        speed_obs  = self.min_speed + (self.max_speed - self.min_speed) * min(1.0, max(0.0, obs_factor))
        heading_error = target_angle - FRONT_ANGLE
        steering_est  = self.Kp * heading_error
        speed_turn    = self.max_speed / (1.0 + self.curve_gain * abs(steering_est))
        speed         = max(self.min_speed, min(speed_obs, speed_turn))

        # 6. Stocker + réinitialiser delta_yaw pour la fast_loop
        self.last_target_angle = target_angle
        self.last_speed        = speed
        self.delta_yaw         = 0.0   # remise à zéro — la fast_loop repart de 0

        # Publier immédiatement (sans attendre la fast_loop)
        self._publish(target_angle, speed)

    # ── Fast loop 50 Hz : correction IMU ─────────────────────────────────
    def _fast_loop(self):
        if self.emergency:
            self.pub_cmd.publish(Twist())
            return
        if self.last_target_angle is None:
            return

        # Le robot a tourné de delta_yaw depuis le dernier scan.
        # La cible (point fixe dans l'espace) apparaît décalée en sens inverse.
        corrected = self.last_target_angle - self.delta_yaw
        self._publish(corrected, self.last_speed)

    # ── Publication ───────────────────────────────────────────────────────
    def _publish(self, target_angle: float, speed: float):
        heading_error = target_angle - FRONT_ANGLE
        steering = self.Kp * heading_error - self.Kd * self.yaw_rate
        steering = max(-1.5, min(1.5, steering))

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
