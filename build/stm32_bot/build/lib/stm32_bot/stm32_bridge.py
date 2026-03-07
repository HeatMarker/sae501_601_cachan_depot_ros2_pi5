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
WHEELBASE = 0.25  
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
        
        # Logique de passage par zéro
        self.target_direction = 1.0          
        self.current_physical_direction = 1.0  
        
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
        
        if speed_mms > 0:
            self.target_direction = 1.0
        elif speed_mms < 0:
            self.target_direction = -1.0
        
        if speed_mms > 3000: speed_mms = 3000
        if speed_mms < -3000: speed_mms = -3000
        
        cmd_steering = msg.angular.z
        servo_value = -cmd_steering * 20.0 
        
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

            if len(self.rx_buffer) > 2 * self.frame_size_rx:
                 self.rx_buffer = self.rx_buffer[-(2 * self.frame_size_rx):]

            while len(self.rx_buffer) >= self.frame_size_rx:
                if self.rx_buffer[0] != 0xAA or self.rx_buffer[1] != 0x55:
                    del self.rx_buffer[0]
                    continue
                
                packet = self.rx_buffer[:self.frame_size_rx]
                if self.crc8_atm(packet[:-1]) == packet[-1]:
                    self.decode_telemetry(packet)
                    self.rx_buffer = self.rx_buffer[self.frame_size_rx:]
                else:
                    del self.rx_buffer[0]

        except Exception as e:
            self.get_logger().warn(f"Serial Error: {e}")

    def decode_telemetry(self, packet):
        try:
            target_ms = 0.0
            error_ms = 0.0

            if self.debug_mode:
                unpacked = struct.unpack('<BBBBIfffB', packet)
                # On lit le hw_time_ms mais on ne l'utilise plus pour ROS
                hw_time_ms = unpacked[4]
                target_ms = unpacked[5]
                actual_ms = unpacked[6] 
                error_ms  = unpacked[7]
            else:
                unpacked = struct.unpack('<BBBBIfB', packet)
                hw_time_ms = unpacked[4]
                actual_ms = unpacked[5]

            # --- HORLOGE UNIQUE ROS 2 ---
            # On utilise directement l'horloge de la Pi pour éviter la dérive avec le Lidar
            odom_time = self.get_clock().now()

            # --- PUBLICATION DEBUG (Si activé) ---
            if self.debug_mode:
                pid_msg = Vector3()
                pid_msg.x = target_ms
                pid_msg.y = actual_ms
                pid_msg.z = error_ms
                self.pub_pid.publish(pid_msg)

            # --- TRAITEMENT DE L'ODOMÉTRIE (Zero-Crossing) ---
            if abs(actual_ms) <= 0.01:
                self.current_physical_direction = self.target_direction
                odom_speed_ms = 0.0 
            else:
                odom_speed_ms = abs(actual_ms) * self.current_physical_direction

            if abs(odom_speed_ms) > 10.0 or not math.isfinite(odom_speed_ms):
                return
            
            odom = Odometry()
            odom.header.stamp = odom_time.to_msg() 
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            odom.twist.twist.linear.x = odom_speed_ms * LINEAR_CORRECTION_FACTOR

            if abs(WHEELBASE) > 0.001:
                angular_vel = (odom_speed_ms / WHEELBASE) * math.tan(self.current_steering_angle)
                odom.twist.twist.angular.z = angular_vel
            else:
                odom.twist.twist.angular.z = 0.0

            odom.twist.covariance = [0.0] * 36
            if abs(odom_speed_ms) < 0.01:
                odom.twist.covariance[0] = 0.001 
                odom.twist.covariance[35] = 0.001 
            else:
                odom.twist.covariance[0] = 0.1  
                odom.twist.covariance[35] = 100.0 

            self.pub_odom.publish(odom)

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