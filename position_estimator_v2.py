# position_estimator.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class PositionEstimator(Node):
    def __init__(self):
        super().__init__('position_estimator')
        # Souscription au topic /odom (Odometry)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        # Publisher pour la position estimée
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/estimation', 10)
        self.get_logger().info('Nœud PositionEstimator démarré, en attente de /odom')
    
    def odom_callback(self, odom_msg):
        # Crée un message PoseWithCovarianceStamped à partir de l'odométrie
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header = odom_msg.header   # reprend le même header (timestamp, frame_id)
        pose_cov_msg.pose = odom_msg.pose       # copie la pose avec covariance
        # Publie la pose estimée sur /estimation
        self.publisher.publish(pose_cov_msg)
        # Log en debug la position actuelle
        pos = odom_msg.pose.pose.position
        self.get_logger().debug(f'Pose actuelle : x={pos.x:.2f}, y={pos.y:.2f}, ref={odom_msg.header.frame_id}')

def main(args=None):
    rclpy.init(args=args)
    node = PositionEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
