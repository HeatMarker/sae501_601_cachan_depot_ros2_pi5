#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# --- PARAMÈTRES ---
MAX_SPEED = 3.0      # m/s
MAX_TURN  = 1.0      # rad/s (~57 deg/s)
STEP_SPEED = 0.2     # Incrément vitesse
STEP_TURN  = 0.1     # Incrément virage

msg = """
---------------------------
PILOTAGE ROBOT STM32 (Fluide)
---------------------------
   Flèche HAUT   : Accélérer
   Flèche BAS    : Ralentir / Reculer
   Flèche GAUCHE : Braquer GAUCHE
   Flèche DROITE : Braquer DROITE

   ESPACE        : STOP D'URGENCE (Recommandé)
   CTRL-C        : Quitter
---------------------------
"""

class CustomTeleop(Node):
    def __init__(self):
        super().__init__('custom_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_speed = 0.0
        self.target_turn = 0.0
        # Sauvegarde des paramètres du terminal d'origine
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        # Cette fonction ne bloque pas, elle regarde s'il y a une touche
        # Si pas de touche en 0.05s, elle renvoie rien.
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b': # Si c'est une séquence d'échappement (flèches)
                key += sys.stdin.read(2)
            return key
        return None

    def constrain(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)

    def run(self):
        print(msg)
        try:
            # ON PASSE EN MODE RAW UNE SEULE FOIS ICI
            tty.setraw(sys.stdin.fileno())
            
            while True:
                key = self.getKey()
                
                # --- LOGIQUE DE CONTRÔLE ---
                if key == '\x1b[A':   # HAUT
                    self.target_speed += STEP_SPEED
                elif key == '\x1b[B': # BAS
                    self.target_speed -= STEP_SPEED
                elif key == '\x1b[D': # GAUCHE
                    self.target_turn -= STEP_TURN
                elif key == '\x1b[C': # DROITE
                    self.target_turn += STEP_TURN
                elif key == ' ':      # ESPACE
                    self.target_speed = 0.0
                    self.target_turn = 0.0
                elif key == '\x03':   # CTRL-C
                    break

                # --- BORNES ---
                self.target_speed = self.constrain(self.target_speed, -MAX_SPEED, MAX_SPEED)
                self.target_turn  = self.constrain(self.target_turn, -MAX_TURN, MAX_TURN)

                # --- PUBLICATION ---
                twist = Twist()
                twist.linear.x = float(self.target_speed)
                twist.angular.z = float(self.target_turn)
                self.pub.publish(twist)

                # --- AFFICHAGE PROPRE ---
                # \r ramène le curseur au début de la ligne sans sauter de ligne
                sys.stdout.write(f"\rVitesse: {self.target_speed:.2f} m/s | Braquage: {self.target_turn:.2f}     ")
                sys.stdout.flush()

        except Exception as e:
            print(e)

        finally:
            # ON STOPPE LE ROBOT AVANT DE QUITTER
            twist = Twist()
            twist.linear.x = 0.0; twist.angular.z = 0.0
            self.pub.publish(twist)
            
            # ON RESTAURE LE TERMINAL NORMALEMENT
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\nSortie propre.")

def main(args=None):
    rclpy.init(args=args)
    teleop = CustomTeleop()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()