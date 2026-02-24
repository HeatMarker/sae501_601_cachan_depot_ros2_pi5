#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty, time

# --- RÉGLAGES INCRÉMENTAUX ---
STEP_SPEED = 0.5      # +0.4 m/s par appui (Bond rapide)
MAX_SPEED  = 3.0      # Vitesse max

STEP_TURN  = 0.25     # Braquage vif (4 coups pour max)
MAX_TURN   = 1.0      # Max braquage

LOOP_HZ = 50          

msg = """
-----------------------------------------
PILOTAGE INCRÉMENTAL (ANTI-LAG)
-----------------------------------------
   HAUT   : Vitesse +0.4 m/s
   BAS    : Vitesse -0.4 m/s
   GAUCHE : Braquage +0.25
   DROITE : Braquage -0.25

   ESPACE : STOP D'URGENCE (Tout à 0)
   CTRL-C : Quitter
-----------------------------------------
"""

class TrackmaniaTeleop(Node):
    def __init__(self):
        super().__init__('trackmania_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_speed = 0.0
        self.current_turn = 0.0
        self.settings = termios.tcgetattr(sys.stdin)

    def run(self):
        print(msg)
        try:
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                # --- SECTION ANTI-LAG (VIDANGE DU BUFFER) ---
                # On lit TOUTES les touches accumulées depuis la dernière boucle
                # au lieu d'une seule. Ça empêche l'overflow.
                while True:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
                    if not rlist: 
                        break # Buffer vide, on sort
                    
                    key = sys.stdin.read(1)
                    if key == '\x1b': # Gestion flèches
                        key += sys.stdin.read(2)

                    # --- LOGIQUE INCRÉMENTALE ---
                    # Appliquée immédiatement pour chaque touche trouvée dans le buffer
                    
                    # VITESSE
                    if key == '\x1b[A':   # HAUT
                        self.current_speed = min(self.current_speed + STEP_SPEED, MAX_SPEED)
                    elif key == '\x1b[B': # BAS
                        self.current_speed = max(self.current_speed - STEP_SPEED, -MAX_SPEED)
                    
                    # DIRECTION
                    elif key == '\x1b[D': # GAUCHE
                        self.current_turn = min(self.current_turn + STEP_TURN, MAX_TURN)
                    elif key == '\x1b[C': # DROITE
                        self.current_turn = max(self.current_turn - STEP_TURN, -MAX_TURN)
                    
                    # STOP
                    elif key == ' ':
                        self.current_speed = 0.0
                        self.current_turn = 0.0
                    
                    elif key == '\x03': # CTRL-C
                        raise KeyboardInterrupt

                # --- PUBLICATION ---
                # On publie la dernière valeur connue (résultat de tous les appuis)
                twist = Twist()
                twist.linear.x = float(self.current_speed)
                twist.angular.z = float(self.current_turn)
                self.pub.publish(twist)

                sys.stdout.write(f"\rVitesse: {self.current_speed:+0.1f} | Direction: {self.current_turn:+0.2f}   ")
                sys.stdout.flush()

                time.sleep(1/LOOP_HZ)

        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"\nErreur: {e}")
        finally:
            self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0; twist.angular.z = 0.0
        self.pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print("\nSortie propre.")

def main(args=None):
    rclpy.init(args=args)
    teleop = TrackmaniaTeleop()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()