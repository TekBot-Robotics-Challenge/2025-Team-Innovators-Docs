import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import heapq
import numpy as np

class PathFinder(Node):
    def __init__(self):
        super().__init__('astar_pathfinder')
        self.get_logger().warn("TEaknmldkzjfmlqskdjfkml")

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.current_pose = None
        self.laser_data = None
        self.timer = self.create_timer(0.1, self.move)

        # Grille pour A* (sera initialisée quand nécessaire)
        self.grid = None
        self.grid_res = 1.0
        self.start = (0, 0)
        self.goal = (9, 9)
        self.path = []
        self.computed_path = False
        self.current_target_index = 0

        # Paramètres pour l'évitement d'obstacles
        self.obstacle_distance_threshold = 0.6  # Distance minimale pour détecter un obstacle
        self.safe_distance = 0.4  # Distance de sécurité
        self.max_linear_speed = 0.25
        self.max_angular_speed = 0.6
        
        # États du robot
        self.robot_state = "MOVING_FORWARD"  # MOVING_FORWARD, AVOIDING_OBSTACLE
        self.avoidance_start_time = None
        self.last_pose_before_avoidance = None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        self.laser_data = msg

    def get_yaw(self):
        if self.current_pose is None:
            return 0.0
        q = self.current_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def detect_obstacles(self):
        """Détecte les obstacles et retourne les informations nécessaires pour l'évitement"""
        if self.laser_data is None:
            return False, False, False, 10.0

        ranges = np.array(self.laser_data.ranges)
        ranges = np.where(np.isinf(ranges), 10.0, ranges)  # Remplacer inf par une grande valeur
        ranges = np.where(np.isnan(ranges), 10.0, ranges)  # Remplacer NaN par une grande valeur
        
        # Analyser différentes zones autour du robot
        n_ranges = len(ranges)
        
        # Zone avant (plus étroite pour une meilleure précision)
        front_start = max(0, n_ranges//2 - 15)
        front_end = min(n_ranges, n_ranges//2 + 15)
        front_zone = ranges[front_start:front_end]
        
        # Zones latérales (plus larges)
        left_zone = ranges[front_end:front_end + 40]
        right_zone = ranges[max(0, front_start - 40):front_start]

        # Vérifier les obstacles
        front_obstacle = np.any(front_zone < self.obstacle_distance_threshold)
        left_clear = np.mean(left_zone) > self.safe_distance
        right_clear = np.mean(right_zone) > self.safe_distance
        
        # Calculer la distance minimale à l'avant
        min_front_distance = np.min(front_zone)
        
        return front_obstacle, left_clear, right_clear, min_front_distance

    def initialize_grid(self):
        """Initialise la grille pour A* avec une configuration de base"""
        # Grille 10x10 avec quelques obstacles
        self.grid = [
            [0,0,0,0,0,0,0,1,1,1],
            [0,1,1,1,1,0,0,0,0,1],
            [0,0,0,0,1,0,1,1,0,0],
            [1,1,1,0,1,0,0,1,1,0],
            [0,0,1,0,0,0,1,0,0,0],
            [0,0,1,1,1,0,0,0,1,0],
            [1,0,0,0,1,1,1,0,1,0],
            [0,1,1,0,0,0,0,0,1,0],
            [0,0,0,0,1,1,1,1,1,0],
            [1,1,1,0,0,0,0,0,0,0],
        ]
        
        # Définir le point de départ basé sur la position actuelle
        if self.current_pose:
            self.start = (
                int(round(self.current_pose.position.y / self.grid_res)),
                int(round(self.current_pose.position.x / self.grid_res))
            )
            # S'assurer que le start est dans la grille
            self.start = (
                max(0, min(len(self.grid)-1, self.start[0])),
                max(0, min(len(self.grid[0])-1, self.start[1]))
            )
        else:
            self.start = (0, 0)

    def calculate_avoidance_velocity(self):
        """Calcule la vitesse d'évitement basée sur les données du laser"""
        front_obstacle, left_clear, right_clear, min_distance = self.detect_obstacles()
        
        cmd = Twist()
        
        if not front_obstacle:
            return None
            
        # Obstacle détecté, choisir la direction d'évitement
        if left_clear and right_clear:
            # Les deux côtés sont libres, choisir le côté avec plus d'espace
            ranges = np.array(self.laser_data.ranges)
            left_space = np.mean(ranges[len(ranges)//2:])
            right_space = np.mean(ranges[:len(ranges)//2])
            
            if left_space > right_space:
                cmd.angular.z = self.max_angular_speed * 0.6
            else:
                cmd.angular.z = -self.max_angular_speed * 0.6
                
        elif left_clear and not right_clear:
            cmd.angular.z = self.max_angular_speed * 0.6
            
        elif right_clear and not left_clear:
            cmd.angular.z = -self.max_angular_speed * 0.6
            
        else:
            # Aucun côté n'est libre, reculer et tourner
            cmd.linear.x = -0.15
            cmd.angular.z = self.max_angular_speed * 0.8
            
        # Ajuster la vitesse linéaire en fonction de la distance
        if min_distance > self.safe_distance and (left_clear or right_clear):
            cmd.linear.x = max(cmd.linear.x, self.max_linear_speed * 0.2)
        
        return cmd

    def move(self):
        if self.current_pose is None or self.laser_data is None:
            return

        cmd = Twist()

        # Vérifier les obstacles
        front_obstacle, left_clear, right_clear, min_distance = self.detect_obstacles()

        if front_obstacle:
            # Mode évitement d'obstacles
            if self.robot_state == "MOVING_FORWARD":
                self.robot_state = "AVOIDING_OBSTACLE"
                self.last_pose_before_avoidance = self.current_pose
                self.get_logger().warn("Warning: Obstacle détecté ! Évitement en cours...")
                self.initialize_grid()  # Initialiser la grille avec la position actuelle
                self.path = self.astar(self.start, self.goal)
                if self.path:
                    self.computed_path = True
                    self.current_target_index = 0
                    self.get_logger().info(f"Good! Chemin trouvé (A*): {self.path}")
                else:
                    self.get_logger().error("❌ Aucun chemin trouvé !")
                    self.robot_state = "MOVING_FORWARD"
                    return
            
            avoidance_cmd = self.calculate_avoidance_velocity()
            if avoidance_cmd:
                self.cmd_pub.publish(avoidance_cmd)
                return
        else:
            # Pas d'obstacle, revenir à l'avance tout droit
            if self.robot_state == "AVOIDING_OBSTACLE":
                self.robot_state = "MOVING_FORWARD"
                self.computed_path = False
                self.path = []
                self.current_target_index = 0
                self.get_logger().info("Good!!! Obstacle évité, reprise de l'avance tout droit.")

        # Comportement selon l'état
        if self.robot_state == "MOVING_FORWARD":
            # Avancer tout droit
            cmd.linear.x = self.max_linear_speed
            cmd.angular.z = 0.0
            
        elif self.robot_state == "AVOIDING_OBSTACLE" and self.path:
            # Suivi du chemin A*
            target_cell = self.path[self.current_target_index]
            target_x = target_cell[1] * self.grid_res
            target_y = target_cell[0] * self.grid_res

            # Calculer la distance et l'angle vers la cible
            dx = target_x - self.current_pose.position.x
            dy = target_y - self.current_pose.position.y
            distance = math.hypot(dx, dy)
            
            if distance < 0.3:  # Seuil de proximité
                self.current_target_index += 1
                if self.current_target_index >= len(self.path):
                    self.robot_state = "MOVING_FORWARD"
                    self.computed_path = False
                    self.path = []
                    self.get_logger().info("Chemin d'évitement terminé.")
                else:
                    self.get_logger().info(f"Point {self.current_target_index}/{len(self.path)} atteint")
                return

            # Calculer l'angle de direction
            target_angle = math.atan2(dy, dx)
            current_angle = self.get_yaw()
            angle_diff = target_angle - current_angle
            
            # Normaliser l'angle entre -pi et pi
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # Contrôle proportionnel
            if abs(angle_diff) > 0.2:  # Si l'angle est trop grand, tourner d'abord
                cmd.linear.x = 0.1
                cmd.angular.z = max(-self.max_angular_speed, 
                                   min(self.max_angular_speed, 3.0 * angle_diff))
            else:
                # Avancer vers la cible
                cmd.linear.x = min(self.max_linear_speed, distance * 0.5)
                cmd.angular.z = max(-self.max_angular_speed * 0.5, 
                                   min(self.max_angular_speed * 0.5, 2.0 * angle_diff))

        self.cmd_pub.publish(cmd)

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def astar(self, start, goal):
        if self.grid is None:
            return []

        rows, cols = len(self.grid), len(self.grid[0])
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start))
        came_from = {}
        cost_so_far = {start: 0}

        directions = [(-1,0),(1,0),(0,-1),(0,1)]

        while open_set:
            _, cost, current = heapq.heappop(open_set)

            if current == goal:
                break

            for dr, dc in directions:
                nr, nc = current[0] + dr, current[1] + dc
                neighbor = (nr, nc)
                if 0 <= nr < rows and 0 <= nc < cols and self.grid[nr][nc] == 0:
                    new_cost = cost + 1
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, new_cost, neighbor))
                        came_from[neighbor] = current

        # Reconstituer le chemin
        path = []
        current = goal
        while current != start:
            if current not in came_from:
                return []  # Aucun chemin trouvé
            path.insert(0, current)
            current = came_from[current]
        path.insert(0, start)
        return path

def main(args=None):
    rclpy.init(args=args)
    node = PathFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
