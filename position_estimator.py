import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class PositionEstimator(Node):
    def __init__(self):
        super().__init__('position_estimator')
        
        # Create a subscriber for the odometry data from Gazebo
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Create a publisher for publishing the estimated pose
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/estimated_pose', 10)

    def odom_callback(self, msg):
        # Extract the position and orientation from the Odometry message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Create a PoseWithCovarianceStamped message to publish the estimated pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'  # Can change if needed
        
        # Copy the position and orientation to the PoseWithCovarianceStamped message
        pose_msg.pose.pose.position = position
        pose_msg.pose.pose.orientation = orientation
        
        # You can optionally add covariance for more accurate estimations
        # pose_msg.pose.covariance = [0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0.1]
        
        # Publish the pose
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Create the position estimator node
    node = PositionEstimator()
    
    # Spin the node to keep receiving and publishing messages
    rclpy.spin(node)
    
    # Shutdown the node when done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
