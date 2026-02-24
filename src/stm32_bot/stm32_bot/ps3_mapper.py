#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class PS3Mapper(Node):
    def __init__(self):
        super().__init__('ps3_mapper')
        
        # --- DÉCLARATION DU PARAMÈTRE ---
        # Si on ne donne pas d'argument au lancement, la vitesse sera de 3.0 m/s par défaut
        self.declare_parameter('max_speed', 3.0)
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info("Mapper PS3 'Trackmania' lancé (X = Deadman, R2 = Gaz, L2 = Frein)")
    
    def joy_callback(self, msg):
        twist = Twist()

        # On récupère la vitesse max actuelle depuis les paramètres ROS 2
        current_max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        # --- SÉCURITÉ (DEADMAN) SUR LA CROIX (Bouton 0) ---
        if msg.buttons[0] == 1:
            # --- VITESSE LINEAIRE (Gâchettes L2/R2) ---
            # Sur PS3/Linux, les gâchettes vont de 1.0 (repos) à -1.0 (enfoncé)
            # On convertit ça en 0.0 à 1.0
            r2_val = (1.0 - msg.axes[5]) / 2.0  # Gaz
            l2_val = (1.0 - msg.axes[2]) / 2.0  # Frein / Marche arrière

            # Vitesse = (Gaz - Frein) * La nouvelle variable max_speed
            twist.linear.x = float((r2_val - l2_val) * current_max_speed)

            # --- DIRECTION (Joystick Gauche Horizontal - Axe 0) ---
            # Inversion du sens : on met un signe '-' pour que Gauche = Gauche
            twist.angular.z = msg.axes[0] * 1.0
        else:
            # Si on lâche la Croix, on stop tout
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PS3Mapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
