#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import serial
import struct
import math
import time

# --- CONFIGURATION PHYSIQUE ---
WHEELBASE = 0.257  # TT-02 standard (à affiner si besoin)
MAX_STEER_ANGLE_RAD = 0.35 

# FACTEUR DE CORRECTION LINEAIRE
LINEAR_CORRECTION_FACTOR = 1.0 

# --- CONFIGURATION SÉRIE ---
REG_SERVO  = 0x00
REG_MOTOR  = 0x01
REG_PID_KP = 0x03
REG_PID_KI = 0x04
REG_PID_KD = 0x05

# TAILLES DE TRAMES
FRAME_SIZE_NORMAL = 13 # 5 (Header) + 4 (Time) + 3 (1 Float: speed) + 1 (CRC)
FRAME_SIZE_DEBUG  = 21 # 5 (Header) + 4 (Time) + 12 (3 Floats) + 1 (CRC)

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        # --- PARAMÈTRES ---
        self.declare_parameter('port', '/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3170365A3334-if00')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('debug_pid', False)
        self.declare_parameter('ackermann_mode', False)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.debug_mode = self.get_parameter('debug_pid').get_parameter_value().bool_value

        self.frame_size_rx = FRAME_SIZE_DEBUG if self.debug_mode else FRAME_SIZE_NORMAL

        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
            mode_str = "Debug PID + Wi-Fi Tuning" if self.debug_mode else "Production (Odom via Horloge ROS)"
            self.get_logger().info(f"Connecté sur {port} - Mode: {mode_str}")
        except Exception as e:
            self.get_logger().error(f"Erreur Serial: {e}")

        # --- SUBSCRIBERS ---
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos)
        
        if self.debug_mode:
            self.create_subscription(Vector3, '/pid_gains', self.pid_gains_callback, 10)
            self.pub_pid = self.create_publisher(Vector3, '/pid_debug', 10)
        
        # --- PUBLISHERS ---
        self.pub_odom = self.create_publisher(Odometry, '/wheel/odom', 10)

        self.rx_buffer = bytearray()
        self.current_steering_angle = 0.0

        self.last_speed_mms = None
        self.last_servo_value = None

        # Dernière vitesse mesurée par l'encodeur STM32 (en m/s)
        self.last_measured_vx_mps = 0.0

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

    def pid_gains_callback(self, msg: Vector3):
        if not self.debug_mode: return
        self.send_command(REG_PID_KP, int(msg.x * 100.0))
        time.sleep(0.01) 
        self.send_command(REG_PID_KI, int(msg.y * 100.0))
        time.sleep(0.01)
        self.send_command(REG_PID_KD, int(msg.z * 100.0))
        self.get_logger().info(f"Nouveaux PID appliqués -> P:{msg.x}, I:{msg.y}, D:{msg.z}")

    def cmd_vel_callback(self, msg: Twist):
        speed_mms = msg.linear.x * 1000.0

        if speed_mms > 3000: speed_mms = 3000
        if speed_mms < -3000: speed_mms = -3000

        if self.get_parameter('ackermann_mode').value:
            vx_ref = max(abs(msg.linear.x), 0.05)
            steering_rad = math.atan(WHEELBASE * msg.angular.z / vx_ref)
            steering_rad = max(min(steering_rad, MAX_STEER_ANGLE_RAD), -MAX_STEER_ANGLE_RAD)
            servo_value = -math.degrees(steering_rad)
        else:
            servo_value = -msg.angular.z * 20.0
            steering_rad = math.radians(-servo_value)

        if servo_value > 20: servo_value = 20
        if servo_value < -20: servo_value = -20

        self.current_steering_angle = steering_rad

        speed_mms = round(speed_mms)
        servo_value = round(servo_value)

        if speed_mms != self.last_speed_mms or servo_value != self.last_servo_value:
            self.send_command(REG_MOTOR, speed_mms)
            time.sleep(0.005)
            self.send_command(REG_SERVO, servo_value)
            self.last_speed_mms = speed_mms
            self.last_servo_value = servo_value

        self.send_command(REG_MOTOR, speed_mms)
        self.send_command(REG_SERVO, servo_value)

    def read_serial_loop(self):
        # Lire les octets disponibles dans le buffer série
        if self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))
            except Exception as e:
                self.get_logger().warn(f"Serial Error: {e}", throttle_duration_sec=5.0)

        # Éviter la croissance infinie du buffer
        if len(self.rx_buffer) > 6 * FRAME_SIZE_NORMAL:
            self.rx_buffer = self.rx_buffer[-(6 * FRAME_SIZE_NORMAL):]

        # Parser toutes les trames complètes disponibles
        while len(self.rx_buffer) >= FRAME_SIZE_NORMAL:
            if self.rx_buffer[0] != 0xAA or self.rx_buffer[1] != 0x55:
                del self.rx_buffer[0]
                continue

            frame = bytes(self.rx_buffer[:FRAME_SIZE_NORMAL])
            if self.crc8_atm(frame[:-1]) == frame[-1]:
                _, _, ftype, _, _ts, speed = struct.unpack('<BBBBIf', frame[:-1])
                if ftype == 0x02 and math.isfinite(speed) and abs(speed) <= 10.0:
                    self.last_measured_vx_mps = speed
                self.rx_buffer = self.rx_buffer[FRAME_SIZE_NORMAL:]
            else:
                del self.rx_buffer[0]

        # Publier l'odométrie avec la vitesse réelle encodeur (m/s)
        vx = self.last_measured_vx_mps

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.twist.twist.linear.x = vx * LINEAR_CORRECTION_FACTOR

        if abs(WHEELBASE) > 0.001:
            odom.twist.twist.angular.z = (vx / WHEELBASE) * math.tan(self.current_steering_angle)
        else:
            odom.twist.twist.angular.z = 0.0

        odom.twist.covariance = [0.0] * 36
        if abs(vx) < 0.001:
            odom.twist.covariance[0]  = 0.001
            odom.twist.covariance[35] = 0.001
        else:
            odom.twist.covariance[0]  = 0.1
            odom.twist.covariance[35] = 100.0

        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()