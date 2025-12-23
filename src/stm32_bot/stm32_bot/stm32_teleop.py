#!/usr/bin/env python3
##
# @file stm32_teleop.py
# @brief Téléopération clavier SIMPLE.
# @details Plus d'affichage dynamique, juste le contrôle.
# @author SCHWAGER Jérôme
# @date 2025

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# --- CONFIGURATION CONSTANTES ---
SPEED_STEP = 0.05    # +50 mm/s
TURN_STEP = 0.3      # ~4 degrés
MAX_SPEED = 1.0      # 1000 mm/s
MAX_TURN = 1.0       # ~20 degrés

## Menu d'affichage fixe
msg = """
--------------------------------
CONTROLE STM32 (KEEP-ALIVE)
--------------------------------
z : Vitesse +50 mm/s
s : Vitesse -50 mm/s
q : Braquage +6 deg (Gauche)
d : Braquage -6 deg (Droite)

ESPACE : ARRET TOTAL (0)
CTRL-C : Quitter
--------------------------------
(Pas d'affichage de la vitesse en temps réel pour garder le terminal propre)
"""

class CustomTeleop(Node):
    def __init__(self):
        super().__init__('stm32_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.0
        self.turn = 0.0
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        """Lecture clavier non bloquante"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def pub_twist(self):
        """Publie la commande ROS (Silencieux)"""
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.turn
        self.publisher_.publish(twist)
        
        # J'ai supprimé les print ici pour éviter le spam

    def run(self):
        print(msg) # On affiche le menu une fois au début
        try:
            while True:
                key = self.get_key()
                
                # --- LOGIQUE ---
                if key == 'z':
                    self.speed = min(self.speed + SPEED_STEP, MAX_SPEED)
                elif key == 's':
                    self.speed = max(self.speed - SPEED_STEP, -MAX_SPEED)
                elif key == 'd':
                    self.turn = min(self.turn + TURN_STEP, MAX_TURN)
                elif key == 'q':
                    self.turn = max(self.turn - TURN_STEP, -MAX_TURN)
                elif key == ' ':
                    self.speed = 0.0
                    self.turn = 0.0
                elif key == '\x03': # CTRL+C
                    break

                # --- MISE A JOUR ---
                self.pub_twist()

        except Exception as e:
            print(e)

        finally:
            # Stop propre
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\n")

def main(args=None):
    rclpy.init(args=args)
    node = CustomTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()