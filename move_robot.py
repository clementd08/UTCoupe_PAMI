import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # Publie toutes les 0.1 secondes

        # Crée une instance de message Twist
        self.msg = Twist()

        # Vitesse linéaire dans l'axe X (avant)
        self.msg.linear.x = 0.5  # Vitesse en m/s

        # Pas de rotation (vitesse angulaire)
        self.msg.angular.z = 0.0

    def publish_velocity(self):
        self.publisher.publish(self.msg)
        self.get_logger().info('Publishing velocity: %f m/s', self.msg.linear.x)

def main(args=None):
    rclpy.init(args=args)
    move_robot = MoveRobot()
    rclpy.spin(move_robot)

    # Destroy the node explicitly
    move_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
