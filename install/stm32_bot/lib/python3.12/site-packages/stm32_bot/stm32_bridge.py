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

# --- CONFIGURATION ---
REG_SERVO  = 0x00
REG_MOTOR  = 0x01
FRAME_SIZE_RX = 13 

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        # Paramètres
        self.declare_parameter('port', '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3170365A3334-if00')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.ser = None
        try:
            # timeout=0 est important pour ne pas bloquer
            self.ser = serial.Serial(port, baud, timeout=0)
            self.get_logger().info(f"Connecté sur {port} (Mode Anti-Lag)")
        except Exception as e:
            self.get_logger().error(f"Erreur Serial: {e}")

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos)
        self.pub_odom = self.create_publisher(Odometry, '/wheel/odom', 10)

        self.last_time = self.get_clock().now()
        self.rx_buffer = bytearray()
        
        # On tourne à 50Hz pour laisser respirer le CPU
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
        steering_deg = msg.angular.z * 20.0 
        
        if speed_mms > 3000: speed_mms = 3000
        if speed_mms < -3000: speed_mms = -3000
        if steering_deg > 20: steering_deg = 20
        if steering_deg < -20: steering_deg = -20

        self.send_command(REG_MOTOR, speed_mms)
        self.send_command(REG_SERVO, steering_deg)

    def read_serial_loop(self):
        if not self.ser or not self.ser.is_open: return
        try:
            # 1. Lecture complète
            if self.ser.in_waiting > 0:
                self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))

            # 2. ANTI-LAG DOUX : On ne garde que les 2 derniers paquets max
            # Si on a trop de données, c'est qu'on est en retard, donc on coupe.
            if len(self.rx_buffer) > 2 * FRAME_SIZE_RX:
                 self.rx_buffer = self.rx_buffer[-(2 * FRAME_SIZE_RX):]

            # 3. Décodage
            while len(self.rx_buffer) >= FRAME_SIZE_RX:
                # Recherche du header 0xAA 0x55
                if self.rx_buffer[0] != 0xAA or self.rx_buffer[1] != 0x55:
                    del self.rx_buffer[0]
                    continue
                
                # On a un candidat
                packet = self.rx_buffer[:FRAME_SIZE_RX]
                
                # Vérif CRC
                if self.crc8_atm(packet[:-1]) == packet[-1]:
                    self.decode_telemetry(packet)
                    # On retire le paquet traité et on continue (au cas où il y en a un autre)
                    self.rx_buffer = self.rx_buffer[FRAME_SIZE_RX:]
                    # Astuce : On ne break PAS ici. Si la STM32 a envoyé 2 positions,
                    # on veut les traiter toutes les deux pour que l'odom soit fluide.
                else:
                    # CRC Faux, on vire juste l'octet de tête
                    del self.rx_buffer[0]

        except Exception as e:
            self.get_logger().warn(f"Serial Error: {e}")

    def decode_telemetry(self, packet):
        try:
            unpacked = struct.unpack('<BBBBIfB', packet)
            actual_speed_ms = unpacked[5] 

            if abs(actual_speed_ms) > 10.0 or not math.isfinite(actual_speed_ms):
                return

            current_time = self.get_clock().now()
            
            # Note : Grâce au nettoyage du buffer, 'dt' sera plus grand (ex: 0.1s au lieu de 0.02s)
            # si on a sauté des paquets. C'est EXACTEMENT ce qu'on veut :
            # Vitesse * Grand DT = La bonne distance parcourue pendant le lag.
            
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.twist.twist.linear.x = actual_speed_ms
            odom.twist.twist.angular.z = 0.0 

            # Covariance Dynamique (Verrouillage à l'arrêt)
            odom.twist.covariance = [0.0] * 36
            if abs(actual_speed_ms) < 0.01:
                odom.twist.covariance[0] = 0.001 
                odom.twist.covariance[35] = 0.001 
            else:
                odom.twist.covariance[0] = 0.1 
                odom.twist.covariance[35] = 100.0

            self.pub_odom.publish(odom)
            self.last_time = current_time # Mise à jour du temps APRES publication

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