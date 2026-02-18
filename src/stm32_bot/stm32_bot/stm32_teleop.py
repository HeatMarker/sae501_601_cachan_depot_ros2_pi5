#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty, time

# --- RÉGLAGES PHYSIQUES ---
MAX_SPEED = 3.0       # m/s
MAX_TURN  = 1.0       # rad/s
ACCEL_RATE = 0.01     # Plus nerveux à l'accélération
DECEL_RATE = 0.01     # Frein moteur
TURN_RATE  = 0.01     # Vitesse de braquage
RETURN_RATE = 0.01    # Retour au centre rapide

LOOP_HZ = 50          
KEY_TIMEOUT = 0.6     # Pour la détection de touche enfoncée
STOP_TIMEOUT = 3.0    # Temps avant décélération automatique (inactivité totale)

msg = """
-----------------------------------------
PILOTAGE ARCADE FIXÉ (Mode Régulateur)
-----------------------------------------
   Maintenir HAUT   : Accélérer
   Maintenir BAS    : Reculer / Freiner
   Maintenir GAUCHE : Braquer GAUCHE
   Maintenir DROITE : Braquer DROITE

   LÂCHER TOUT      : Maintient la vitesse pendant 3s
   ESPACE           : STOP D'URGENCE (Vitesse -> 0)
   CTRL-C           : Quitter
-----------------------------------------
"""

class TrackmaniaTeleop(Node):
    def __init__(self):
        super().__init__('trackmania_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_speed = 0.0
        self.current_turn = 0.0
        self.settings = termios.tcgetattr(sys.stdin)
        self.last_key_time = time.time()
        self.active_key = None

    def getKey(self):
        key = None
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
            if rlist:
                s = sys.stdin.read(1)
                if s == '\x1b':
                    s += sys.stdin.read(2)
                key = s
            else:
                break
        return key

    def approach(self, current, target, rate):
        if current < target:
            return min(current + rate, target)
        elif current > target:
            return max(current - rate, target)
        return target

    def run(self):
        print(msg)
        try:
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                key = self.getKey()
                now = time.time()

                if key:
                    if key == '\x03': break 
                    self.active_key = key
                    self.last_key_time = now # On réinitialise le chrono à chaque appui
                
                # Touche active (enjambe la latence clavier)
                is_pressed = (now - self.last_key_time) < KEY_TIMEOUT
                key_to_process = self.active_key if is_pressed else None

                # --- GESTION VITESSE (LOGIQUE MODIFIÉE) ---
                if key_to_process == '\x1b[A':   # HAUT
                    self.current_speed = self.approach(self.current_speed, MAX_SPEED, ACCEL_RATE)
                elif key_to_process == '\x1b[B': # BAS
                    self.current_speed = self.approach(self.current_speed, -MAX_SPEED, ACCEL_RATE)
                else:
                    # Si aucune touche n'est pressée, on vérifie le délai d'inactivité totale
                    if (now - self.last_key_time) > STOP_TIMEOUT:
                        # Plus de 3s sans rien toucher : on décélère vers 0
                        self.current_speed = self.approach(self.current_speed, 0.0, DECEL_RATE)
                    else:
                        # Entre 0.6s et 3s : on maintient la vitesse actuelle
                        pass

                # --- GESTION DIRECTION (CONSERVÉE) ---
                if key_to_process == '\x1b[D':   # GAUCHE
                    self.current_turn = self.approach(self.current_turn, -MAX_TURN, TURN_RATE)
                elif key_to_process == '\x1b[C': # DROITE
                    self.current_turn = self.approach(self.current_turn, +MAX_TURN, TURN_RATE)
                else:
                    # La direction revient toujours au centre quand on lâche
                    self.current_turn = self.approach(self.current_turn, 0.0, RETURN_RATE)

                # --- SÉCURITÉ ESPACE ---
                if key == ' ':
                    self.current_speed = 0.0
                    self.current_turn = 0.0

                # --- PUBLICATION ---
                twist = Twist()
                twist.linear.x = float(self.current_speed)
                twist.angular.z = float(self.current_turn)
                self.pub.publish(twist)

                sys.stdout.write(f"\rVitesse: {self.current_speed:+0.2f} | Direction: {self.current_turn:+0.2f}   ")
                sys.stdout.flush()

                time.sleep(1/LOOP_HZ)

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