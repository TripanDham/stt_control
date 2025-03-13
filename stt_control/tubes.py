#!/usr/bin/env python3
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import matplotlib.pyplot as plt
import tf_transformations
import tf2_ros
import tf2_geometry_msgs
import time
import os
from control_msgs.action import FollowJointTrajectory

os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so'

class DrivePublisherNode():
    def __init__(self):
        # Create a regular ROS 2 node
        self._node = Node('tubes')
        self.logger = self._node.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # Define a constant angular speed for steering correction (bang-bang)
        self.steering_speed = 0.5  # rad/s; adjust as needed
        self.heading_tolerance = 0.1  # radians

        # TF buffer and listener for transform lookup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self._node)

        # Subscribers (using your topics)
        self.odom_subscriber = self._node.create_subscription(
            Odometry, '/a/odom', self.odom_callback, 1, callback_group=self.callback_group)
        self.joint_states_subscriber = self._node.create_subscription(
            JointState, '/a/stretch/joint_states', self.joint_states_callback, 1, callback_group=self.callback_group)

        # Publisher for base velocity and action client for joint trajectory
        self.base_pub = self._node.create_publisher(Twist, '/a/stretch/cmd_vel', 10)
        self.trajectory_client = ActionClient(
            self._node, FollowJointTrajectory, '/stretch_a_controller/follow_joint_trajectory')

        # Limits (for reference)
        self.z_lower_lim = 0.54
        self.z_upper_lim = 1.22
        self.arm_lower_lim = 0.00
        self.arm_upper_lim = 0.12

        # State variables
        self.base_pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.lift_pos = 0.0
        self.arm_pos = 0.0

        # Flags for received messages
        self.odom_received = False
        self.joint_received = False

        # Obtain initial end-effector (EE) position via TF
        x_base = self.base_pose[0]
        y_base = self.base_pose[1]
        try:
            base_arm_transform = self.tf_buffer.lookup_transform('link_aruco_top_wrist', 'base_link', rclpy.time.Time())
            delX_start = base_arm_transform.transform.translation.x
            delY_start = base_arm_transform.transform.translation.y
            delZ_start = base_arm_transform.transform.translation.z
        except Exception as e:
            delX_start = delY_start = delZ_start = 0.0
            self.logger.info("Could not receive initial TF from base to wrist.")
        x_ee_start = x_base + delX_start
        y_ee_start = y_base + delY_start
        z_ee_start = delZ_start

        # Define start and goal sets for EE (three tubes: x, y, z)
        self.start_set = np.array([
            [x_ee_start - 0.1, x_ee_start + 0.1],
            [y_ee_start - 0.1, y_ee_start + 0.1],
            [z_ee_start - 0.05, z_ee_start + 0.05]
        ])
        # Example goal in EE coordinates (modify as needed)
        self.goal_set = np.array([
            [4.0 - 0.1, 4.0 + 0.1],
            [1.0 - 0.1, 1.0 + 0.1],
            [0.6 - 0.05, 0.6 + 0.05]
        ])
        self.t_final = 20

        self.logger.info(f"Start EE tube: {self.start_set}, Goal EE tube: {self.goal_set}")

        # Precompute the goal point (center of the goal tube) and define threshold for stopping
        self.goal_point = np.array([
            np.mean(self.goal_set[0]),
            np.mean(self.goal_set[1]),
            np.mean(self.goal_set[2])
        ])
        self.goal_threshold = 0.05  # meters
        self.goal_reached = False

        # Create spatiotemporal tubes and compute the center trajectory and its derivative
        self.stt_val, self.center_path = self.create_tube(self.start_set, self.goal_set, self.t_final)
        self.center_vel = self.differentiate_center_path(self.center_path, self.time_arr, self.t_final)
        self.plot_planned_path(self.center_path, self.stt_val)

        input("Press Enter to approve trajectory and start robot motion...")

        self.start_time = round(self._node.get_clock().now().nanoseconds / 1e9, 4)

        # Data logging arrays for visualization
        self.ee_x_arr = []
        self.ee_y_arr = []
        self.ee_z_arr = []
        self.des_x_arr = []
        self.des_y_arr = []
        self.des_z_arr = []
        self.cmd_vel_arr = []
        self.steer_arr = []
        self.vlift_arr = []

    def odom_callback(self, odom_msg):
        # Update current base pose from odometry
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        orientation = odom_msg.pose.pose.orientation
        theta = tf_transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])[2]
        self.base_pose = np.array([x, y, theta])
        self.odom_received = True

        # Run the control loop on each odom message
        self.control_loop()

    def joint_states_callback(self, joint_states):
        self.lift_pos = joint_states.position[joint_states.name.index('joint_lift')]
        self.arm_pos = joint_states.position[joint_states.name.index('joint_arm_l0')]
        self.joint_received = True

    def differentiate_center_path(self, center_path, time_arr, t_final):
        # Compute the derivative using one-step finite differences
        vel = np.zeros_like(center_path)
        dt = time_arr[1] - time_arr[0]
        vel[:, 1:] = (center_path[:, 1:] - center_path[:, :-1]) / dt
        vel[:, 0] = vel[:, 1]
        return vel

    def create_tube(self, start_set, goal_set, t_final):
        dt = 0.001
        self.time_arr = np.arange(0, t_final, dt)
        # X-axis tube
        rho_l_x = np.array([start_set[0, 0] + (goal_set[0, 0] - start_set[0, 0]) * np.tanh(t / (t_final - t)) for t in self.time_arr])
        rho_u_x = np.array([start_set[0, 1] + (goal_set[0, 1] - start_set[0, 1]) * np.tanh(t / (t_final - t)) for t in self.time_arr])
        center_x = (rho_l_x + rho_u_x) / 2
        # Y-axis tube
        rho_l_y = np.array([start_set[1, 0] + (goal_set[1, 0] - start_set[1, 0]) * np.tanh(t / (t_final - t)) for t in self.time_arr])
        rho_u_y = np.array([start_set[1, 1] + (goal_set[1, 1] - start_set[1, 1]) * np.tanh(t / (t_final - t)) for t in self.time_arr])
        center_y = (rho_l_y + rho_u_y) / 2
        # Z-axis tube
        rho_l_z = np.array([start_set[2, 0] + (goal_set[2, 0] - start_set[2, 0]) * np.tanh(t / (t_final - t)) for t in self.time_arr])
        rho_u_z = np.array([start_set[2, 1] + (goal_set[2, 1] - start_set[2, 1]) * np.tanh(t / (t_final - t)) for t in self.time_arr])
        center_z = (rho_l_z + rho_u_z) / 2

        stt_val = np.array([self.time_arr, rho_l_x, rho_u_x, rho_l_y, rho_u_y, rho_l_z, rho_u_z])
        center_path = np.array([center_x, center_y, center_z])
        self.logger.info("Spatiotemporal tubes created.")
        return stt_val, center_path

    def get_ref_center(self, t):
        # Interpolate the center trajectory at time t
        center_x = np.interp(t, self.time_arr, self.center_path[0])
        center_y = np.interp(t, self.time_arr, self.center_path[1])
        center_z = np.interp(t, self.time_arr, self.center_path[2])
        return np.array([center_x, center_y, center_z])

    def control_loop(self):
        # Compute elapsed time
        current_t = round(self._node.get_clock().now().nanoseconds / 1e9, 4) - self.start_time

        # Get current base pose and orientation
        x_base, y_base, theta = self.base_pose

        # Get TF from base_link to EE
        try:
            transform = self.tf_buffer.lookup_transform('link_aruco_top_wrist', 'base_link', rclpy.time.Time())
            delX = transform.transform.translation.x
            delY = transform.transform.translation.y
            delZ = transform.transform.translation.z
        except Exception as e:
            self.logger.info("Could not get TF from base to wrist; using zeros.")
            delX = delY = delZ = 0.0

        # Compute current EE position (global, approximate)
        x_ee = x_base + delX
        y_ee = y_base + delY
        z_ee = delZ
        self.pose_ee = np.array([x_ee, y_ee, z_ee])

        # Check if the goal is reached
        distance_to_goal = np.linalg.norm(self.pose_ee - self.goal_point)
        if distance_to_goal < self.goal_threshold:
            self.logger.info("Goal reached. Stopping robot and unsubscribing from odometry.")
            self.goal_reached = True
            self.zero_vel()
            self._node.destroy_subscription(self.odom_subscriber)
            return

        # Get desired EE position and velocity from the precomputed trajectory
        des_pos = self.get_ref_center(current_t)
        des_vel_x = np.interp(current_t, self.time_arr, self.center_vel[0])
        des_vel_y = np.interp(current_t, self.time_arr, self.center_vel[1])
        des_vel_z = np.interp(current_t, self.time_arr, self.center_vel[2])
        des_vel = np.array([des_vel_x, des_vel_y, des_vel_z])

        # Log desired and actual positions for plotting
        self.des_x_arr.append(des_pos[0])
        self.des_y_arr.append(des_pos[1])
        self.des_z_arr.append(des_pos[2])
        self.ee_x_arr.append(x_ee)
        self.ee_y_arr.append(y_ee)
        self.ee_z_arr.append(z_ee)

        # For planar motion: compute desired heading from the goal using transforms
        # Instead of using the desired velocity from the trajectory, compute the desired heading directly:
        desired_heading = np.arctan2(self.goal_point[1] - y_base, self.goal_point[0] - x_base)
        heading_error = desired_heading - theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        # Use a fixed angular speed to correct heading (bang–bang control)
        if abs(heading_error) > self.heading_tolerance:
            cmd_vel_angular = self.steering_speed * np.sign(heading_error)
        else:
            cmd_vel_angular = 0.0

        # For linear motion, still use the desired XY velocity from the trajectory
        cmd_vel_linear = np.linalg.norm(des_vel[:2])
        # Vertical command is taken directly from the desired Z velocity
        cmd_vel_z = des_vel[2]

        # Log commands for plotting
        self.cmd_vel_arr.append(cmd_vel_linear)
        self.steer_arr.append(cmd_vel_angular)
        self.vlift_arr.append(cmd_vel_z)

        # Publish base command for planar motion
        drive_msg = Twist()
        drive_msg.linear.x = cmd_vel_linear
        drive_msg.angular.z = cmd_vel_angular
        self.base_pub.publish(drive_msg)

        # Send vertical (lift) command via the trajectory client (arm remains passive)
        arm_speed = {
            'joint_lift': cmd_vel_z,
            'joint_arm_l0': 0.0,
            'joint_arm_l1': 0.0,
            'joint_arm_l2': 0.0,
            'joint_arm_l3': 0.0
        }
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0.0).to_msg()
        point.velocities = [arm_speed[j] for j in arm_speed]
        joint_names = list(arm_speed.keys())
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        trajectory_goal.trajectory.joint_names = joint_names
        trajectory_goal.trajectory.points = [point]
        self.trajectory_client.send_goal_async(trajectory_goal)

        self.logger.info(f"t={current_t:.2f}: lin_vel={cmd_vel_linear:.3f}, steer={cmd_vel_angular:.3f}, lift_vel={cmd_vel_z:.3f}")

    def zero_vel(self):
        drive_msg = Twist()
        drive_msg.linear.x = 0.0
        drive_msg.angular.z = 0.0
        self.base_pub.publish(drive_msg)
        arm_speed = {
            'joint_lift': 0.0,
            'joint_arm_l0': 0.0,
            'joint_arm_l1': 0.0,
            'joint_arm_l2': 0.0,
            'joint_arm_l3': 0.0
        }
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0.0).to_msg()
        point.velocities = [arm_speed[j] for j in arm_speed]
        joint_names = list(arm_speed.keys())
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        trajectory_goal.trajectory.joint_names = joint_names
        trajectory_goal.trajectory.points = [point]
        self.trajectory_client.send_goal_async(trajectory_goal)

    def plot_planned_path(self, center_path, stt_val):
        time_arr = self.time_arr
        center_x = center_path[0]
        center_y = center_path[1]
        center_z = center_path[2]
        rho_l_x = stt_val[1]
        rho_u_x = stt_val[2]
        rho_l_y = stt_val[3]
        rho_u_y = stt_val[4]
        rho_l_z = stt_val[5]
        rho_u_z = stt_val[6]

        fig, axs = plt.subplots(2, 2, figsize=(12, 10))
        axs[0, 0].plot(time_arr, center_x, label='Planned EE x (Center)', color='r')
        axs[0, 0].plot(time_arr, rho_l_x, label='EE x Lower', linestyle='--')
        axs[0, 0].plot(time_arr, rho_u_x, label='EE x Upper', linestyle='--')
        axs[0, 0].set_title('EE x vs Time')
        axs[0, 0].set_xlabel('Time')
        axs[0, 0].set_ylabel('End-effector x')
        axs[0, 0].legend()

        axs[0, 1].plot(time_arr, center_y, label='Planned EE y (Center)', color='r')
        axs[0, 1].plot(time_arr, rho_l_y, label='EE y Lower', linestyle='--')
        axs[0, 1].plot(time_arr, rho_u_y, label='EE y Upper', linestyle='--')
        axs[0, 1].set_title('EE y vs Time')
        axs[0, 1].set_xlabel('Time')
        axs[0, 1].set_ylabel('End-effector y')
        axs[0, 1].legend()

        axs[1, 0].plot(time_arr, center_z, label='Planned EE z (Center)', color='r')
        axs[1, 0].plot(time_arr, rho_l_z, label='EE z Lower', linestyle='--')
        axs[1, 0].plot(time_arr, rho_u_z, label='EE z Upper', linestyle='--')
        axs[1, 0].set_title('EE z vs Time')
        axs[1, 0].set_xlabel('Time')
        axs[1, 0].set_ylabel('End-effector z')
        axs[1, 0].legend()

        axs[1, 1].axis('off')
        plt.tight_layout()
        plt.show(block=True)

    def plot(self):
        fig, axs = plt.subplots(2, 2, figsize=(12, 10))
        axs[0, 0].plot(np.array(self.ee_x_arr), np.array(self.ee_y_arr), label='Actual EE XY Path', color='b')
        axs[0, 0].set_title('End-effector XY Path')
        axs[0, 0].set_xlabel('EE x')
        axs[0, 0].set_ylabel('EE y')
        axs[0, 0].legend()
        axs[0, 0].axis('equal')
        axs[0, 0].grid(True)

        axs[0, 1].plot(self.ee_z_arr, label='Actual EE z', color='g')
        axs[0, 1].set_title('End-effector z vs Time')
        axs[0, 1].set_xlabel('Time steps')
        axs[0, 1].set_ylabel('EE z')
        axs[0, 1].legend()

        axs[1, 0].plot(self.cmd_vel_arr, label='Linear cmd_vel', color='purple')
        axs[1, 0].set_title('Commanded Linear Velocity vs Time')
        axs[1, 0].set_xlabel('Time steps')
        axs[1, 0].set_ylabel('Linear Velocity')
        axs[1, 0].legend()

        axs[1, 1].plot(self.steer_arr, label='Angular steer', color='orange')
        axs[1, 1].plot(self.vlift_arr, label='Lift Velocity', color='brown')
        axs[1, 1].set_title('Angular Command & Lift Velocity vs Time')
        axs[1, 1].set_xlabel('Time steps')
        axs[1, 1].set_ylabel('Command')
        axs[1, 1].legend()

        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = DrivePublisherNode()
    try:
        rclpy.spin(node._node)
    except KeyboardInterrupt:
        node.logger.info("Node stopped by user.")
        node.zero_vel()
        node.plot()
    finally:
        node._node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
