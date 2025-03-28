#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty

class PathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher_node")
        self.get_logger().info("Starting path publisher node...")
        
        self.path_publisher = self.create_publisher(Path, "/path", 10)
        self.goal_publisher = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)

        self.path = Path()
        self.path.header.frame_id = "odom"
        
        self.create_test_path()
        self.publish_path()

    def create_test_path(self):
        # Create a simple path with 4 waypoints
        waypoints = [
            (0.0, 0.0),  # Start position
            (1.0, 1.0),  # Move diagonally
            (2.0, 0.0),  # Move horizontally
            (3.0, 1.0)   # End position
        ]

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "odom"
            
            # Ensure the values are of type float
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0

            # Set the orientation as a quaternion (no rotation here)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            self.path.poses.append(pose)


    def publish_path(self):
        self.get_logger().info("Publishing test path...")
        self.path_publisher.publish(self.path)

        # Create a goal for the robot to reach
        self.publish_goal(self.path.poses[-1])  # Use the last point in the path as the goal

    def publish_goal(self, goal):
        self.goal_publisher.publish(goal)

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
