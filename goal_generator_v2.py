# goal_generator.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
import random
import math

class GoalGenerator(Node):
    def __init__(self):
        super().__init__('goal_generator')
        # Publisher sur le topic /goal pour les objectifs
        self.publisher = self.create_publisher(PoseStamped, '/goal', 10)
        # Timer pour générer un objectif périodiquement (toutes les 10 s)
        self.timer = self.create_timer(10.0, self.timer_callback)
        self.get_logger().info('Nœud GoalGenerator démarré : publie un nouvel objectif toutes les 10 secondes')
    
    def timer_callback(self):
        # Génère une cible aléatoire (x, y, yaw) dans la zone [-5, 5]
        x = random.uniform(-5.0, 5.0)
        y = random.uniform(-5.0, 5.0)
        yaw = random.uniform(-math.pi, math.pi)
        # Crée le message PoseStamped pour la cible
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'  # Cadre de référence de l'objectif
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        # Position de l'objectif
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        # Orientation : conversion du yaw en quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        q.w = math.cos(yaw / 2)
        goal_msg.pose.orientation = q
        # Publie l'objectif sur /goal
        self.publisher.publish(goal_msg)
        # Log l’objectif généré
        self.get_logger().info(f'Nouvel objectif publié : x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
