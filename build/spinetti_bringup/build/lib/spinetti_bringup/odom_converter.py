import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped

class OdomConverter(Node):
    def __init__(self):
        super().__init__('odom_converter')

        # Souscriptions
        self.speed_sub = self.create_subscription(Float32, '/speed', self.speed_callback, 10)
        ##self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Publication pour l'EKF
        self.odom_pub = self.create_publisher(TwistWithCovarianceStamped, '/velocity/ackermann', 10)

        # Variables internes
        self.current_speed = 0.0
        ##self.current_gyro_z = 0.0

    def speed_callback(self, msg):
        self.current_speed = msg.data
        self.publish_twist()

    def imu_callback(self, msg):
        self.current_gyro_z = msg.angular_velocity.z

    def publish_twist(self):
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link' # Très important

        # Vitesse linéaire (venant de l'arbre)
        msg.twist.twist.linear.x = self.current_speed
        ## Vitesse angulaire (venant du gyro)
        ##msg.twist.twist.angular.z = self.current_gyro_z

        # On définit une confiance (Covariance)
        # Plus la valeur est petite, plus ROS fera confiance au capteur
        covariance = [0.0] * 36
        for i in range(0, 36, 7): # Remplit la diagonale (0, 7, 14, ...)
            covariance[i] = 1e-6
        covariance[0] = 0.01  # Confiance en la vitesse X
        covariance[35] = 0.01 # Confiance en la rotation Z
        msg.twist.covariance = covariance

        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()