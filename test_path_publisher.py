#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import sqrt
import random

class RandomGoalPublisher(Node):
    def __init__(self):
        super().__init__("random_goal_publisher_node")
        self.get_logger().info("Starting random goal publisher node...")

        # Publisher pour envoyer des objectifs
        self.goal_publisher = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        # Abonnement à l'odométrie pour obtenir la position actuelle du robot
        self.odom_subscriber = self.create_subscription(Odometry, "/odom", self.receive_odometry, 10)

        self.current_position = None
        self.target_goal = None
        
        # Timer qui vérifie la progression et publie un nouvel objectif si besoin
        self.create_timer(2.0, self.check_and_publish_goal)

    def receive_odometry(self, msg):
        # Récupère la position actuelle du robot
        self.current_position = msg.pose.pose.position

    def generate_random_goal(self):
        # Génère des coordonnées aléatoires dans un espace défini
        x = random.uniform(-5.0, 5.0)  # Plage de -5 à 5 pour X
        y = random.uniform(-5.0, 5.0)  # Plage de -5 à 5 pour Y
        
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Orientation neutre (aucune rotation)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        
        return goal

    def publish_goal(self):
        # Génère et publie un nouvel objectif aléatoire
        self.target_goal = self.generate_random_goal()
        self.goal_publisher.publish(self.target_goal)
        self.get_logger().info(
            f"Publishing new random goal: ({self.target_goal.pose.position.x}, {self.target_goal.pose.position.y})"
        )

    def check_and_publish_goal(self):
        # Si la position actuelle n'est pas connue, on ne fait rien
        if self.current_position is None:
            return
        
        # Si aucun objectif n'est défini, on en publie un nouveau
        if self.target_goal is None:
            self.publish_goal()
            return
        
        # Calculer la distance entre la position actuelle et l'objectif
        distance = sqrt(
            (self.target_goal.pose.position.x - self.current_position.x) ** 2 +
            (self.target_goal.pose.position.y - self.current_position.y) ** 2
        )
        
        # Seuil pour considérer que le robot a atteint l'objectif (ajustez ce seuil si nécessaire)
        if distance < 0.2:
            self.get_logger().info(
                f"Goal reached! Current: ({self.current_position.x}, {self.current_position.y}), "
                f"Target: ({self.target_goal.pose.position.x}, {self.target_goal.pose.position.y})"
            )
            # Publier un nouveau but
            self.publish_goal()

    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalPublisher()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
