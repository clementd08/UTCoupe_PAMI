# controller.py
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        # Gains PID pour la commande
        self.Kp_lin = 0.5
        self.Ki_lin = 0.0
        self.Kd_lin = 0.0
        self.Kp_ang = 1.0
        self.Ki_ang = 0.0
        self.Kd_ang = 0.0
        # État courant du robot (position et orientation)
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        # Objectif courant (position et orientation)
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        # Erreurs précédentes et intégrales (pour le PID)
        self.prev_dist_error = None
        self.integral_dist_error = 0.0
        self.prev_angle_error = None
        self.integral_angle_error = 0.0
        self.last_time = self.get_clock().now()
        # Seuils de tolérance pour considérer l'objectif atteint
        self.position_tolerance = 0.1      # en mètres
        self.orientation_tolerance = 0.1   # en radians
        # Souscriptions aux topics /estimation et /goal
        self.create_subscription(PoseWithCovarianceStamped, '/estimation', self.estimation_callback, 10)
        self.create_subscription(PoseStamped, '/goal', self.goal_callback, 10)
        # Publisher pour les commandes de vitesse (/cmd_vel)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Nœud Controller démarré, en attente des messages /estimation et /goal')
    
    def estimation_callback(self, msg):
        # Met à jour la pose estimée actuelle du robot
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Calcule le yaw actuel à partir du quaternion (supposant roll=pitch ~ 0)
        q = msg.pose.pose.orientation
        self.current_yaw = 2 * math.atan2(q.z, q.w)
        # Si aucun objectif n'est défini, on ne fait rien
        if self.goal_x is None:
            return
        # Calcule le pas de temps depuis la dernière itération
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        if dt == 0:
            dt = 1e-6
        self.last_time = current_time
        # Calcule les erreurs de distance et d'angle par rapport à la cible
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_error = math.sqrt(dx*dx + dy*dy)
        # Angle absolu vers la position de la cible
        target_angle = math.atan2(dy, dx)
        # Choisit la référence d'orientation en fonction de la proximité de la cible
        if distance_error > self.position_tolerance:
            # En route vers la cible : on s'aligne vers la position cible (angle de visée)
            angle_error = self.normalize_angle(target_angle - self.current_yaw)
        else:
            # Proche de la cible : on ajuste l'orientation finale demandée
            angle_error = self.normalize_angle(self.goal_yaw - self.current_yaw)
            distance_error = 0.0  # on considère l'erreur de position nulle une fois assez proche
        # Calcul PID pour la distance (commande linéaire)
        self.integral_dist_error += distance_error * dt
        if self.prev_dist_error is None:
            self.prev_dist_error = distance_error
        dist_error_rate = (distance_error - self.prev_dist_error) / dt
        self.prev_dist_error = distance_error
        linear_cmd = (self.Kp_lin * distance_error +
                      self.Ki_lin * self.integral_dist_error +
                      self.Kd_lin * dist_error_rate)
        # Calcul PID pour l'orientation (commande angulaire)
        self.integral_angle_error += angle_error * dt
        if self.prev_angle_error is None:
            self.prev_angle_error = angle_error
        angle_error_rate = (angle_error - self.prev_angle_error) / dt
        self.prev_angle_error = angle_error
        angular_cmd = (self.Kp_ang * angle_error +
                       self.Ki_ang * self.integral_angle_error +
                       self.Kd_ang * angle_error_rate)
        # Ralentit la vitesse linéaire si l'angle à corriger est important (évite une approche trop rapide en courbe)
        if distance_error > self.position_tolerance:
            linear_cmd *= math.cos(angle_error)
            if linear_cmd < 0.0:
                linear_cmd = 0.0  # ne pas reculer
        # Limite les commandes pour respecter des vitesses maximales
        max_lin = 0.5   # m/s (vitesse linéaire max)
        max_ang = 1.0   # rad/s (vitesse angulaire max)
        if linear_cmd > max_lin:
            linear_cmd = max_lin
        if angular_cmd > max_ang:
            angular_cmd = max_ang
        if angular_cmd < -max_ang:
            angular_cmd = -max_ang
        # Arrête le robot si la cible est atteinte (position ET orientation dans les tolérances)
        if distance_error <= self.position_tolerance and abs(angle_error) < self.orientation_tolerance:
    if not hasattr(self, 'goal_reached_time'):
        # Marque le temps d'arrivée
        self.goal_reached_time = self.get_clock().now()
        self.get_logger().info('Objectif atteint → pause...')
        return  # Ne publie rien pendant cette itération

    elapsed = (self.get_clock().now() - self.goal_reached_time).nanoseconds * 1e-9
    if elapsed < 2.0:  # 2 secondes de pause
        # Envoie une vitesse nulle pendant la pause
        twist = Twist()
        self.cmd_pub.publish(twist)
        return
    else:
        # Fin de la pause → reset pour accepter une nouvelle cible
        self.get_logger().info('Pause terminée.')
        self.goal_reached_time = None
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        self.prev_dist_error = None
        self.prev_angle_error = None
        self.integral_dist_error = 0.0
        self.integral_angle_error = 0.0
        return

        
        # Publie la commande sur /cmd_vel
        twist = Twist()
        twist.linear.x = linear_cmd
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_cmd
        self.cmd_pub.publish(twist)
        # Logs des erreurs et des vitesses commandées
        self.get_logger().info(f'Erreur distance = {distance_error:.2f}, Erreur angle = {angle_error:.2f}, ' +
                               f'cmd_vel → lin = {linear_cmd:.2f}, ang = {angular_cmd:.2f}')
    
    def goal_callback(self, msg):
        # Met à jour la cible courante lors de la réception d'un nouveau /goal
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        # Extrait le yaw de l'objectif à partir du quaternion
        q = msg.pose.orientation
        self.goal_yaw = 2 * math.atan2(q.z, q.w)
        self.get_logger().info(f'Nouvel objectif reçu : x={self.goal_x:.2f}, y={self.goal_y:.2f}, yaw={self.goal_yaw:.2f}')
    
    def normalize_angle(self, angle: float) -> float:
        # Normalise un angle en radians dans l'intervalle [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
