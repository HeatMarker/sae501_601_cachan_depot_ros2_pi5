#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import struct
import math
import time

# --- CONFIGURATION PHYSIQUE ---
WHEELBASE = 0.25  
MAX_STEER_ANGLE_RAD = 0.35 

# FACTEUR DE CORRECTION LINEAIRE (Pour le test du mur)
# 1.0 = Pas de changement
# > 1.0 = Augmente la vitesse odométrique (si RViz est trop lent par rapport au réel)
# < 1.0 = Diminue la vitesse odométrique (si RViz est trop rapide)
LINEAR_CORRECTION_FACTOR = 1.0 

# --- CONFIGURATION SÉRIE ---
REG_SERVO  = 0x00
REG_MOTOR  = 0x01
FRAME_SIZE_RX = 13 

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        self.declare_parameter('port', '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3170365A3334-if00')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
            self.get_logger().info(f"Connecté sur {port} (Mode Calibration Odom + Zero-Crossing)")
        except Exception as e:
            self.get_logger().error(f"Erreur Serial: {e}")

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos)
        self.pub_odom = self.create_publisher(Odometry, '/wheel/odom', 10)

        self.last_time = self.get_clock().now()
        self.rx_buffer = bytearray()
        self.current_steering_angle = 0.0  
        
        # --- LOGIQUE DE PASSAGE PAR ZÉRO ---
        self.target_direction = 1.0          # Ce que demande la manette
        self.current_physical_direction = 1.0  # Le vrai sens physique actuel
        
        self.create_timer(1.0 / 30.0, self.read_serial_loop)

    def crc8_atm(self, data):
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80: crc = (crc << 1) ^ 0x07
                else: crc = (crc << 1)
                crc &= 0xFF
        return crc

    def send_command(self, register, value_int):
        if not self.ser or not self.ser.is_open: return
        val_safe = max(min(int(value_int), 32000), -32000)
        if val_safe < 0: val_safe += 65536
        payload = [0x00 | (register & 0x7F), val_safe & 0xFF, (val_safe >> 8) & 0xFF]
        frame = bytearray(payload + [self.crc8_atm(payload)])
        try: self.ser.write(frame)
        except: pass

    def cmd_vel_callback(self, msg: Twist):
        speed_mms = msg.linear.x * 1000.0
        
        # --- MISE À JOUR DE LA CONSIGNE ---
        if speed_mms > 0:
            self.target_direction = 1.0
        elif speed_mms < 0:
            self.target_direction = -1.0
        # Si speed_mms == 0, on garde la dernière direction voulue en mémoire
        
        if speed_mms > 3000: speed_mms = 3000
        if speed_mms < -3000: speed_mms = -3000
        
        cmd_steering = msg.angular.z
        servo_value = cmd_steering * 20.0 
        
        if servo_value > 20: servo_value = 20
        if servo_value < -20: servo_value = -20

        self.current_steering_angle = cmd_steering * MAX_STEER_ANGLE_RAD

        self.send_command(REG_MOTOR, speed_mms)
        self.send_command(REG_SERVO, servo_value)

    def read_serial_loop(self):
        if not self.ser or not self.ser.is_open: return
        try:
            if self.ser.in_waiting > 0:
                self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))

            if len(self.rx_buffer) > 2 * FRAME_SIZE_RX:
                 self.rx_buffer = self.rx_buffer[-(2 * FRAME_SIZE_RX):]

            while len(self.rx_buffer) >= FRAME_SIZE_RX:
                if self.rx_buffer[0] != 0xAA or self.rx_buffer[1] != 0x55:
                    del self.rx_buffer[0]
                    continue
                
                packet = self.rx_buffer[:FRAME_SIZE_RX]
                if self.crc8_atm(packet[:-1]) == packet[-1]:
                    self.decode_telemetry(packet)
                    self.rx_buffer = self.rx_buffer[FRAME_SIZE_RX:]
                else:
                    del self.rx_buffer[0]

        except Exception as e:
            self.get_logger().warn(f"Serial Error: {e}")

    def decode_telemetry(self, packet):
        try:
            unpacked = struct.unpack('<BBBBIfB', packet)
            raw_speed_ms = unpacked[5] 

            # --- APPLICATION DU PASSAGE PAR ZÉRO ---
            if abs(raw_speed_ms) <= 0.01:
                # La voiture est physiquement arrêtée. 
                # On l'autorise à changer de sens si la manette le demande.
                self.current_physical_direction = self.target_direction
                actual_speed_ms = 0.0  # On force un beau zéro bien propre
            else:
                # La voiture roule encore (inertie), on garde le signe de son mouvement actuel
                actual_speed_ms = abs(raw_speed_ms) * self.current_physical_direction

            if abs(actual_speed_ms) > 10.0 or not math.isfinite(actual_speed_ms):
                return

            current_time = self.get_clock().now()
            
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            # 1. Vitesse Linéaire AVEC FACTEUR DE CORRECTION ET ZERO-CROSSING
            odom.twist.twist.linear.x = actual_speed_ms * LINEAR_CORRECTION_FACTOR

            # 2. Vitesse Angulaire (Calcul Ackermann - Conservé mais ignoré par l'EKF)
            if abs(WHEELBASE) > 0.001:
                angular_vel = (actual_speed_ms / WHEELBASE) * math.tan(self.current_steering_angle)
                odom.twist.twist.angular.z = angular_vel
            else:
                odom.twist.twist.angular.z = 0.0

            odom.twist.covariance = [0.0] * 36
            if abs(actual_speed_ms) < 0.01:
                odom.twist.covariance[0] = 0.001 
                odom.twist.covariance[35] = 0.001 
            else:
                odom.twist.covariance[0] = 0.1  
                odom.twist.covariance[35] = 100.0 

            self.pub_odom.publish(odom)
            self.last_time = current_time

        except struct.error:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()