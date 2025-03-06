#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
import matplotlib.pyplot as plt
from threading import Lock

# =============================================
# Improved RRT* Path Planner
# =============================================
class RRTStarPlanner:
    def __init__(self, start, goal, obstacles, rand_area, max_iter=1000, eps=0.5):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles
        self.rand_area = rand_area
        self.max_iter = max_iter
        self.eps = eps
        self.goal_bias = 0.1  # 10% chance to sample goal directly

    def plan(self):
        nodes = [{'coord': self.start, 'cost': 0, 'parent': None}]
        goal_reached = False
        
        for _ in range(self.max_iter):
            # Sample with goal biasing
            if np.random.random() < self.goal_bias:
                q_rand = self.goal
            else:
                q_rand = np.array([
                    np.random.uniform(self.rand_area[0]), 
                    np.random.uniform(self.rand_area[1])
                ])
            
            # Find nearest node
            nearest = min(nodes, key=lambda n: np.linalg.norm(n['coord'] - q_rand))
            q_near = nearest['coord']
            
            # Steer towards q_rand
            direction = q_rand - q_near
            dist = np.linalg.norm(direction)
            if dist > 0:
                q_new = q_near + (direction / dist) * min(dist, self.eps)
            else:
                continue
                
            # Collision check
            if self.no_collision(q_near, q_new):
                # Find near nodes within radius
                radius = 2.0 * self.eps * math.sqrt(math.log(len(nodes)+1)/len(nodes))
                near_nodes = [n for n in nodes 
                            if np.linalg.norm(n['coord'] - q_new) < radius]
                
                # Choose best parent
                min_cost = nearest['cost'] + np.linalg.norm(q_new - q_near)
                best_parent = nearest
                
                for node in near_nodes:
                    if node['cost'] + np.linalg.norm(q_new - node['coord']) < min_cost:
                        if self.no_collision(node['coord'], q_new):
                            min_cost = node['cost'] + np.linalg.norm(q_new - node['coord'])
                            best_parent = node
                
                # Create new node
                new_node = {
                    'coord': q_new,
                    'cost': min_cost,
                    'parent': best_parent
                }
                nodes.append(new_node)
                
                # Rewire near nodes
                for node in near_nodes:
                    if new_node['cost'] + np.linalg.norm(node['coord'] - q_new) < node['cost']:
                        if self.no_collision(q_new, node['coord']):
                            node['parent'] = new_node
                            node['cost'] = new_node['cost'] + np.linalg.norm(node['coord'] - q_new)
                
                # Check goal proximity
                if np.linalg.norm(q_new - self.goal) < self.eps:
                    goal_reached = True
                    break

        # Extract path if goal reached
        path = []
        if goal_reached:
            node = nodes[-1]
            while node is not None:
                path.append(node['coord'])
                node = node['parent']
            path.reverse()
            
            # Add final goal point
            path.append(self.goal)
            
            # Smooth path
            path = self.smooth_path(path)
        
        return path

    def smooth_path(self, path):
        if len(path) < 3:
            return path
            
        smoothed = [path[0]]
        for i in range(1, len(path)-1):
            p_prev = smoothed[-1]
            p_next = path[i+1]
            if self.no_collision(p_prev, p_next):
                continue  # Skip redundant point
            smoothed.append(path[i])
        smoothed.append(path[-1])
        return smoothed

    def no_collision(self, p1, p2):
        for (ox, oy, w, h) in self.obstacles:
            if self.line_rect_intersect(p1, p2, ox, oy, ox+w, oy+h):
                return False
        return True

    def line_rect_intersect(self, p1, p2, xmin, ymin, xmax, ymax):
        # Liang-Barsky line clipping algorithm
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        p = [-dx, dx, -dy, dy]
        q = [p1[0] - xmin, xmax - p1[0], p1[1] - ymin, ymax - p1[1]]
        
        u1 = 0.0
        u2 = 1.0
        
        for i in range(4):
            if p[i] == 0:
                if q[i] < 0:
                    return True  # Parallel line outside
                continue
                
            t = q[i] / p[i]
            if p[i] < 0 and t > u1:
                u1 = t
            elif p[i] > 0 and t < u2:
                u2 = t
                
        return u1 < u2 and not (u2 < 0 or u1 > 1)

# =============================================
# Enhanced Pure Pursuit Controller
# =============================================
class PurePursuitController:
    def __init__(self, lookahead=0.5, max_speed=0.3, k_angular=1.5):
        self.lookahead = lookahead
        self.max_speed = max_speed
        self.k_angular = k_angular
        self.current_target_idx = 0

    def compute_command(self, current_pose, path):
        if len(path) < 2:
            return 0.0, 0.0

        x, y, theta = current_pose
        
        # Find closest path segment
        closest_dist = float('inf')
        target_point = path[-1]
        self.current_target_idx = 0
        
        for i in range(len(path)-1):
            seg_start = np.array(path[i])
            seg_end = np.array(path[i+1])
            point = np.array([x, y])
            
            # Vector math for projection
            vec = seg_end - seg_start
            t = np.dot(point - seg_start, vec) / np.dot(vec, vec)
            t = np.clip(t, 0.0, 1.0)
            projection = seg_start + t*vec
            dist = np.linalg.norm(point - projection)
            
            # Find best lookahead point
            if dist < closest_dist and t < 0.95:
                closest_dist = dist
                lookahead_t = t + self.lookahead/np.linalg.norm(vec)
                if lookahead_t <= 1.0:
                    target_point = seg_start + lookahead_t*vec
                    self.current_target_idx = i
                else:
                    # Move to next segment
                    remaining = self.lookahead - (1.0 - t)*np.linalg.norm(vec)
                    if i+1 < len(path)-1:
                        next_vec = path[i+2] - path[i+1]
                        next_t = remaining / np.linalg.norm(next_vec)
                        target_point = path[i+1] + next_t*next_vec
                        self.current_target_idx = i+1

        # Calculate steering angle
        dx = target_point[0] - x
        dy = target_point[1] - y
        target_angle = math.atan2(dy, dx)
        alpha = target_angle - theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # Normalize
        
        # Calculate commands
        angular = self.k_angular * math.tan(alpha)
        linear = self.max_speed * math.cos(alpha)
        
        return linear, angular

# =============================================
# Main ROS2 Node Implementation
# =============================================
class RRTPurePursuitNode(Node):
    def __init__(self):
        super().__init__('rrt_pure_pursuit_node')
        
        # Navigation state
        self.current_pose = np.array([0.0, 0.0, 0.0])
        self.path = []
        self.initial_pose = None
        self.moving = False
        self.path_lock = Lock()
        
        # Configuration
        self.map_size = 10.0
        self.obstacles = [[2.0, 3.0, 4.0, 5.0]]
        self.goal = [8.0, 5.0]
        
        # ROS2 setup
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)
        
        # Controller and visualization
        self.controller = PurePursuitController(lookahead=0.8, max_speed=0.3, k_angular=1.2)
        self.setup_visualization()

    def setup_visualization(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, self.map_size)
        self.ax.set_ylim(0, self.map_size)
        self.ax.set_aspect('equal')
        self.ax.grid(True)
        self.ax.set_title("RRT* Path Following")
        
        # Draw obstacles
        for obs in self.obstacles:
            self.ax.add_patch(plt.Rectangle(
                (obs[0], obs[1]), obs[2], obs[3], 
                color='gray', alpha=0.5
            ))
        
        # Visualization elements
        self.robot_marker = self.ax.plot([], [], 'go', markersize=12)[0]
        self.path_line = self.ax.plot([], [], 'b-', linewidth=2)[0]
        self.goal_marker = self.ax.plot([], [], 'ro', markersize=10)[0]

    def odom_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'yaw': tf_transformations.euler_from_quaternion([
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ])[2]
            }
            return

        # Calculate relative pose
        current_x = msg.pose.pose.position.x - self.initial_pose['x']
        current_y = msg.pose.pose.position.y - self.initial_pose['y']
        current_yaw = tf_transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])[2] - self.initial_pose['yaw']
        current_yaw = math.atan2(math.sin(current_yaw), math.cos(current_yaw))
        
        self.current_pose = np.array([current_x, current_y, current_yaw])
        self.update_visualization()

    def update_visualization(self):
        # Update robot position
        self.robot_marker.set_data(
            self.current_pose[0], 
            self.current_pose[1]
        )
        
        # Update path display
        if self.path:
            path_x = [p[0] for p in self.path]
            path_y = [p[1] for p in self.path]
            self.path_line.set_data(path_x, path_y)
        
        # Update goal marker
        self.goal_marker.set_data(self.goal[0], self.goal[1])
        
        plt.draw()
        plt.pause(0.001)

    def control_loop(self):
        if self.initial_pose is None:
            return
        
        # Path planning
        if not self.path and not self.moving:
            with self.path_lock:
                planner = RRTStarPlanner(
                    start=self.current_pose[:2].tolist(),
                    goal=self.goal,
                    obstacles=self.obstacles,
                    rand_area=[0, self.map_size]
                )
                self.path = planner.plan()
            
            if self.path:
                self.get_logger().info("Path planning complete")
                response = input("Start navigation? [y/n]: ").lower()
                if response == 'y':
                    self.moving = True
                else:
                    self.path = []

        # Path following
        if self.moving and self.path:
            # Compute control commands
            linear, angular = self.controller.compute_command(self.current_pose, self.path)
            
            # Safety checks
            linear = np.clip(linear, -0.3, 0.3)
            angular = np.clip(angular, -1.5, 1.5)
            
            # Publish command
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self.cmd_vel_pub.publish(cmd)
            
            # Check goal proximity
            goal_dist = np.linalg.norm(self.current_pose[:2] - self.goal)
            if goal_dist < 0.2:
                self.moving = False
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info("Goal reached!")
                self.path = []

def main(args=None):
    rclpy.init(args=args)
    node = RRTPurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
