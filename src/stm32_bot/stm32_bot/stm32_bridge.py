#!/usr/bin/env python3
##
# @file stm32_bridge.py
# @brief Nœud passerelle (Bridge) entre ROS2 et le microcontrôleur STM32.
# @details Ce nœud assure deux fonctions principales :
#          1. Subscriber : Écoute /cmd_vel, convertit les unités (m/s -> mm/s) et envoie les consignes au STM32.
#          2. Publisher : Lit le port série, décode la trame binaire IMU, et publie sur /imu/data et /speed.
# @author SCHWAGER Jérôme
# @date 2025

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import serial
import struct
import time

# --- CONSTANTES PROTOCOLE ---
## Adresse du registre pour le Servo (Braquage)
REG_SERVO = 0x00
## Adresse du registre pour le Moteur (Vitesse)
REG_MOTOR = 0x01

# --- CONFIGURATION ROBOT ---
## Angle de braquage maximum (en degrés) pour la sécurité (Clamp)
MAX_STEERING_ANGLE = 20.0
## Vitesse linéaire maximale (en mm/s) pour la sécurité (Clamp)
MAX_LINEAR_SPEED = 1000.0

class STM32Bridge(Node):
    """
    @brief Classe principale du Bridge STM32.
    @details Gère la communication série bidirectionnelle asynchrone.
    """

    def __init__(self):
        """
        @brief Constructeur du nœud.
        @details Initialise :
                 - La connexion Série (/dev/ttyACM0 par défaut).
                 - Le Subscriber cmd_vel.
                 - Le Publisher imu.
                 - Le Publisher speed.
                 - Le Timer de lecture (50Hz).
        """
        super().__init__('stm32_bridge')

        # --- PARAMETRES ROS ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        ## Port série (récupéré des params ROS)
        port = self.get_parameter('port').get_parameter_value().string_value
        ## Vitesse en bauds (récupéré des params ROS)
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # --- CONNEXION SERIE ---
        try:
            ## Objet Serial pour la communication
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f"Connecté au STM32 sur {port} à {baud} bauds")
        except Exception as e:
            self.get_logger().error(f"ERREUR CRITIQUE SERIE : {e}")
            # On ne quitte pas brutalement pour laisser ROS tourner, mais le node sera inactif
            self.ser = None

        # --- SUBSCRIBER ---
        ## Abonnement aux commandes de vélocité
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # --- PUBLISHER ---
        ## Publication des données de la centrale inertielle
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        ## Publication des données de vitesse
        self.pub_speedometer = self.create_publisher(Float32, '/speed', 10)

        # --- TIMER DE LECTURE ---
        # Appel la boucle de lecture toutes les 0.02s (50Hz)
        self.create_timer(0.02, self.read_serial_loop)

        ## Buffer de réception pour accumuler les octets
        self.rx_buffer = bytearray()
        ## Taille attendue de la trame IMU : Header(2) + TypeLen(2) + Time(4) + Acc(12) + Gyro(12) + Speed(4) + CRC(1)
        self.imu_frame_size = 37

    def crc8_atm(self, data):
        """
        @brief Calcule le CRC8 (Polynôme 0x07).
        @param data Liste ou bytearray des données à vérifier.
        @return L'octet de CRC calculé.
        """
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc = (crc << 1)
                crc &= 0xFF
        return crc

    def send_register(self, addr, value_int):
        """
        @brief Construit et envoie une trame d'écriture au STM32.
        @param addr Adresse du registre (0x00 ou 0x01).
        @param value_int Valeur entière signée (int16).
        """
        if not self.ser:
            self.get_logger().warn("send_register: port série non initialisé, frame non envoyée")
            return

        # Header: Write (bit7=0) | addr
        hdr = 0x00 | (addr & 0x7F)
        
        # Gestion int16 (complément à 2 pour les valeurs négatives)
        if value_int < 0: value_int += 65536
        
        d0 = value_int & 0xFF
        d1 = (value_int >> 8) & 0xFF
        
        payload = [hdr, d0, d1]
        crc = self.crc8_atm(payload)
        frame = bytearray(payload + [crc])
        
        try:
            # Loggable hex dump to help debugging (promoted to INFO so visible sans DEBUG global)
            self.get_logger().info(f"Envoi registre addr=0x{addr:02X} value={value_int} frame={frame.hex()}")
            self.ser.write(frame)
        except Exception as e:
            self.get_logger().warn(f"Erreur écriture série: {e}")

    def cmd_vel_callback(self, msg: Twist):
        """
        @brief Callback appelé à chaque réception de commande ROS.
        @details Convertit les unités standards ROS (SI) vers les unités STM32 (Raw).
        @param msg Message geometry_msgs/Twist (linear.x en m/s, angular.z en rad/s).
        """
        # 1. Conversion Vitesse linéaire X (m/s) -> Moteur (mm/s)
        speed_mms = int(msg.linear.x * 1000.0)
        speed_mms = max(min(speed_mms, int(MAX_LINEAR_SPEED)), int(-MAX_LINEAR_SPEED))
        
        # 2. Conversion Vitesse angulaire Z (rad/s) -> Servo (Degrés)
        # Ratio actuel : 1.0 rad/s (~57°/s) => commande servo 20°
        steering_deg = int(msg.angular.z * 20.0)
        steering_deg = max(min(steering_deg, int(MAX_STEERING_ANGLE)), int(-MAX_STEERING_ANGLE))

        # Debug: log conversions avant envoi
        self.get_logger().info(f"cmd_vel reçue: linear.x={msg.linear.x:.3f} m/s -> {speed_mms} mm/s, angular.z={msg.angular.z:.3f} -> {steering_deg} deg")
        self.send_register(REG_MOTOR, speed_mms)
        self.send_register(REG_SERVO, steering_deg)

    def read_serial_loop(self):
        """
        @brief Boucle de lecture périodique du port série.
        @details Lit les octets disponibles, cherche le header 0xAA 0x55, vérifie le CRC
                 et déclenche la publication si la trame est valide.
        """
        if not self.ser: return
        
        try:
            if self.ser.in_waiting > 0:
                self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))

                while len(self.rx_buffer) >= self.imu_frame_size:
                    # Cherche le header 0xAA 0x55 (Signature du STM32)
                    if self.rx_buffer[0] == 0xAA and self.rx_buffer[1] == 0x55:
                        packet = self.rx_buffer[:self.imu_frame_size]
                        
                        # Vérif CRC (le dernier octet est le CRC)
                        if self.crc8_atm(packet[:-1]) == packet[-1]:
                            self.publish_imu(packet)
                            del self.rx_buffer[:self.imu_frame_size]
                        else:
                            # Mauvais CRC, on jette 1 octet et on réessaie (glissement)
                            del self.rx_buffer[:1]
                    else:
                        # Pas le bon header, on décale d'un octet pour chercher plus loin
                        del self.rx_buffer[:1]
        except Exception as e:
            self.get_logger().warn(f"Erreur lecture série: {e}")

    def publish_imu(self, packet):
        """
        @brief Décode la trame binaire brute et publie le message ROS Imu.
        @param packet Bytearray contenant la trame complète (33 octets).
        """
        try:
            # Structure binaire (Little Endian <):
            # B=U8 (AA), B=U8 (55), B=Type, B=Len, I=Time(U32)
            # f=float(AccX), f=float(AccY), f=float(AccZ)
            # f=float(GyroX), f=float(GyroY), f=float(GyroZ)
            # f=float(Speed)
            # B=CRC
            unpacked = struct.unpack('<BBBBIf f f f f f f B', packet)
            
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_link" # Repère du robot

            speed = Float32()

            # Conversion Accel: STM32 envoie mm/s², ROS veut m/s²
            imu_msg.linear_acceleration.x = unpacked[5] / 1000.0
            imu_msg.linear_acceleration.y = unpacked[6] / 1000.0
            imu_msg.linear_acceleration.z = unpacked[7] / 1000.0

            # Conversion Gyro: STM32 envoie rad/s, ROS veut rad/s (Pas de changement)
            imu_msg.angular_velocity.x = unpacked[8]
            imu_msg.angular_velocity.y = unpacked[9]
            imu_msg.angular_velocity.z = unpacked[10]

            # Vitesse Speedometer : STM32 envoie m/s, ROS veut ? (Pas de changement)
            speed.data = unpacked[11]

            # Note: L'orientation (Quaternion) n'est pas remplie ici.
            # Elle sera calculée par l'EKF (robot_localization) via fusion Gyro/Accel.

            self.pub_imu.publish(imu_msg)
            self.pub_speedometer.publish(speed)

        except Exception as e:
            self.get_logger().warn(f"Erreur décodage structure: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()