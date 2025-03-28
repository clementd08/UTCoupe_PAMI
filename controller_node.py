import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, Quaternion
from std_msgs.msg import Empty
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from math import atan2, pi
from time import time
from controle_robot.utils import dist_point_to_segment_signed
from controle_robot.pid import PID
import math




MAX_ANG_VEL = 1.0

class Controller(Node):
    def __init__(self):
        super().__init__("control_node")

        self.get_logger().info("Starting control node...")

        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.estimate_subscriber = self.create_subscription(PoseWithCovarianceStamped, "/estimation", self.receive_estimate_pose, 10)
        self.path_subscriber = self.create_subscription(Path, "/path", self.receive_path, 10)
        self.is_goal_reached_publisher = self.create_publisher(Empty, "/is_goal_reached", 10)
        self.goal_publisher = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)

        # Target position and current pose
        self.target_pose = np.array([0., 0., 0.])  # x, y, theta
        self.current_pose = np.array([0., 0., 0.])  # x, y, theta

        # Control parameters
        self.max_linear_velocity = 0.15
        self.max_angular_velocity = 0.1
        self.max_angular_velocity_while_moving = 0.4

        self.max_accel_lin = 0.05
        self.max_accel_ang = 0.5
        self.linear_speed, self.angular_speed = 0, 0
        
        self.angle_control_pid = PID(1.0, 0.0, 0.0)
        self.speed_control_pid = PID(1.0, 0.0, 0.0)
        self.dir_correction_pid = PID(1.0, 0.0, 0.0)

        # State machine for goal handling
        self.is_turning = False
        self.is_moving = False

        self.last_time = time()

        self.period = 0.05
        self.create_timer(self.period, self.control_loop)

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path = None  # Initialisation de path

    def receive_estimate_pose(self, msg):
        self.current_pose = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2])

    def publish_goal(self, goal):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "odom"
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = float(0)

        # Utilisez le quaternion comme tuple
        q = self.euler_to_quaternion(0, 0, 0)  # Aucun angle de rotation, orientation nulle
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.goal_publisher.publish(goal_pose)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Calcul du quaternion à partir des angles d'Euler
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

        # Retourner un tuple de valeurs x, y, z, w
        return (qx, qy, qz, qw)

    def get_next_goal_on_path(self, path_msg):
        path = [np.array([pose.pose.position.x, pose.pose.position.y]) for pose in path_msg.poses]
        segs = [(path[i - 1], path[i]) for i in range(1, len(path))]

        min_dist = float('inf')
        nearest_seg = None
        for seg in segs:
            dist = abs(dist_point_to_segment_signed(self.current_pose[:2], seg[0], seg[1]))
            if dist < min_dist and np.linalg.norm(self.current_pose[:2] - seg[1]) > 0.05:
                min_dist = dist
                nearest_seg = seg

        return nearest_seg

    def receive_path(self, path_msg):
        self.get_logger().info("Received path")
        self.path = path_msg
        self.target_pose = self.get_next_goal_on_path(self.path)[1]  # End of the current segment
        self.is_turning = True
        self.is_moving = False

    def control_loop(self):
        if self.path is not None:
            current_segment = self.get_next_goal_on_path(self.path)
            self.target_pose = current_segment[1]  # End of the current segment

        if self.target_pose is None:
            self.get_logger().info("Target pose is none")
            return

        linear_speed, angular_speed = 0.0, 0.0
        error_vector = self.target_pose[:2] - self.current_pose[:2]
        angle_error_rad = atan2(error_vector[1], error_vector[0]) - self.current_pose[2]

        if abs(angle_error_rad) > 0.5:
            self.is_turning = True
            self.is_moving = False

        if self.is_turning:
            # Modulo to ensure angle is between -pi and pi
            if angle_error_rad > pi:
                angle_error_rad -= 2 * pi
            if angle_error_rad < -pi:
                angle_error_rad += 2 * pi

            # If we're close enough to the target angle, we stop turning
            if abs(angle_error_rad) < pi / 180 and not self.is_moving or self.is_moving and abs(angle_error_rad) < 0.001:
                self.get_logger().info("Angle OK")
                self.is_moving = True
                angular_speed = 0.0
            else:
                angular_speed = self.angle_control_pid.update(angle_error_rad, self.period)
                angular_speed = min(self.max_angular_velocity, angular_speed)
                angular_speed = max(-self.max_angular_velocity, angular_speed)

        if self.is_moving:
            if np.linalg.norm(error_vector[:2]) < 0.01:
                self.get_logger().info("Position OK")
                self.is_moving = False
                self.is_turning = False
                self.target_pose = None
                linear_speed = 0.0
                angular_speed = 0.0
                self.is_goal_reached_publisher.publish(Empty())
            else:
                distance_error_m = np.linalg.norm(error_vector[:2])
                linear_speed = self.speed_control_pid.update(distance_error_m, self.period)
                linear_speed = min(self.max_linear_velocity, linear_speed)
                linear_speed = max(-self.max_linear_velocity, linear_speed)

                # Correct direction
                dist = dist_point_to_segment_signed(self.current_pose[:2], current_segment[0], current_segment[1])
                correction = self.dir_correction_pid.update(dist, self.period)
                correction_max = 0.1
                correction = np.clip(correction, -correction_max, correction_max)
                angular_speed += correction

        if self.target_pose is not None:
            self.publish_goal(self.target_pose)

        # Handle acceleration limits
        dt = time() - self.last_time
        self.last_time = time()

        accel_lin = (linear_speed - self.linear_speed) / dt
        accel_ang = (angular_speed - self.angular_speed) / dt

        if accel_lin > self.max_accel_lin:
            linear_speed = self.linear_speed + self.max_accel_lin * dt
        if accel_ang > self.max_accel_ang:
            angular_speed = self.angular_speed + self.max_accel_ang * dt

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

        self.publish_cmd_vel(linear_speed, angular_speed)

    def publish_cmd_vel(self, linear_speed, angular_speed):
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
