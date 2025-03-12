import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
import rclpy.logging
import rclpy.time

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import matplotlib.pyplot as plt
import tf_transformations
import tf2_ros
import tf2_geometry_msgs
import csv
from control_msgs.action import FollowJointTrajectory
import time
import sys
import hello_helpers.hello_misc as hm
import stretch_body.robot as rb
import os

os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so'

class DrivePublisherNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'stt_kinematics','stt_kinematics',wait_for_first_pointcloud=False)
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()
        
        self.odom_subscriber = self.create_subscription(Odometry, '/b/odom', self.odom_callback, 1, callback_group=self.callback_group)

        self.joint_states_subscriber = self.create_subscription(JointState, '/b/stretch/joint_states', qos_profile=1, callback=self.joint_states_callback, callback_group=self.callback_group)

        self.base_pub = self.create_publisher(Twist, '/b/stretch/cmd_vel', 10, callback_group=self.callback_group)

        # self.joint_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_callback, 10)

        self.z_lower_lim = 0.54
        self.z_upper_lim = 1.22

        self.arm_lower_lim = 0.00
        self.arm_upper_lim = 0.12 #the actual arm length is 4x the distance in the joint state
        
        self.base_pose = np.array([0,0,0])
        self.lift_pos = 0
        self.arm_pos = 0
        self.flag = False

        time.sleep(2)

        while not self.flag:
            self.get_logger().info("Not recieved")
        
        self.get_logger().info(f"lift={self.lift_pos}, arm={self.arm_pos}")

        x = self.base_pose[0]
        y = self.base_pose[1]

        self.start_set = np.array([[x-0.1,x+0.1], [y-0.1,y+0.1], [self.lift_pos-0.05, self.lift_pos+0.05], [self.arm_pos-0.005, self.arm_pos+0.005]])
        self.goal_set = np.array([[1-0.1, 1+0.1], [0.2-0.1, 0.2+0.1], [0.8-0.05, 0.8+0.05], [0.1-0.005, 0.1+0.005]])
        self.t_final = 20

        self.get_logger().info(f"start set: {self.start_set}, goal_set={self.goal_set}")

        self.create_tube(self.start_set, self.goal_set, self.t_final)
        
        self.start_time = round(self.get_clock().now().nanoseconds/1e9, 4)
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group = self.callback_group)  # Publish every 0.01 seconds

        self.ulim = 0.3
        self.rlim = 0.4
        self.vlift_lim = 0.1
        self.varm_lim = 0.05

        self.rhod_0 = 1.0
        self.rhoo_0 = 1.0
        
        self.decay = 0.01

        self.rhod_inf = 0.01
        self.rhod_lower = -0.1
        
        self.rhoo_inf = 0.01

        self.rholift_0 = 0.2
        self.rholift_inf = 0.01
        self.rhoarm_0 = 0.05
        self.rhoarm_inf = 0.001

        self.x_arr = []
        self.y_arr = []
        self.lift_arr = []
        self.arm_arr = []
        
        self.theta_arr = []
        self.ed_arr = []
        self.eo_arr = []
        self.el_arr = []
        self.ea_arr = []
        
        self.rhod_arr = []
        self.rhoo_arr = []
        self.rhol_arr = []
        self.rhoa_arr = []

        self.cmd_vel_arr = []
        self.steer_arr = []
        self.lift_vel_arr = []
        self.arm_vel_arr = []
        self.ref_lower_arr = []
        self.ref_upper_arr = []


        self.count = 0

    def odom_callback(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        
        orientation = odom_msg.pose.pose.orientation
        theta = tf_transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        
        self.base_pose = np.array((x,y,theta))

    def joint_states_callback(self, joint_states):
        self.flag = True
        self.joint_states = joint_states
        self.lift_pos = joint_states.position[joint_states.name.index('joint_lift')]
        self.arm_pos = joint_states.position[joint_states.name.index('joint_arm_l0')]


    def timer_callback(self):
        x_base = self.base_pose[0]
        y_base = self.base_pose[1]
        theta = self.base_pose[2]

        base_arm_transform = TransformStamped()
        try:
            base_arm_transform = self.tf_buffer.lookup_transform('link_aruco_top_wrist', 'base_link', rclpy.time.Time())
            delX = base_arm_transform.transform.translation.x
            delY = base_arm_transform.transform.translation.y
            delZ = base_arm_transform.transform.translation.z
        except:
            delX = 0
            delY = 0
            delZ = 0
            self.get_logger().info("Could not recieve TF base to wrist")

        x = x_base + delX
        y = y_base + delY
        z = delZ

        self.pose = np.array([x_base + delX, y_base + delY, delZ])

        t = round(self.get_clock().now().nanoseconds/1e9, 4) - self.start_time
        ref_lower, ref_upper = self.get_ref_pose(t)
        
        ex = ((ref_lower[0]+ref_upper[0])/2 - x)/(ref_upper[0] - ref_lower[0])
        ey = ((ref_lower[1]+ref_upper[1])/2 - y)/(ref_upper[1] - ref_lower[1])

        ed = np.sqrt(ex**2 + ey**2)
        eo = (ex*np.sin(theta) - ey*np.cos(theta)) #ed not divided

        el = (ref_lower[2] + ref_upper[2])/2 - self.lift_pos
        ea = (ref_lower[3] + ref_upper[3])/2 - self.arm_pos

        #forward velocity funnel
        rhod = self.rhod_inf + (self.rhod_0 - self.rhod_inf) * np.exp(-self.decay*t)
        Xid = 2*(ed - 0.5*(rhod + self.rhod_inf))/(rhod - self.rhod_lower)
        epsd = self.transform(Xid) * self.ulim
        cmd_vel = epsd

        #steer funnel
        rhoo = self.rhoo_inf + (self.rhoo_0 - self.rhoo_inf) * np.exp(-self.decay*t)
        Xio = eo/rhoo
        epsr = self.transform(Xio) * self.rlim
        steer = -epsr

        #lift funnel
        rho_lift = self.rholift_inf + (self.rholift_0 - self.rholift_inf) * np.exp(-self.decay*t)
        Xil = el/rho_lift
        epsl = self.transform(Xil) * self.vlift_lim
        v_lift = epsl

        #arm funnel
        rho_arm = self.rhoarm_inf + (self.rhoarm_0 - self.rhoarm_inf) * np.exp(-self.decay*t)
        Xiarm = ea/rho_arm
        epsa = self.transform(Xiarm) * self.varm_lim
        v_arm = epsa

        self.x_arr.append(x)
        self.y_arr.append(y)
        self.ed_arr.append(ed)
        self.rhod_arr.append(rhod)

        self.cmd_vel_arr.append(cmd_vel)
        self.steer_arr.append(steer)

        self.eo_arr.append(eo)
        self.theta_arr.append(theta)
        self.rhoo_arr.append(rhoo)

        self.ea_arr.append(ea)
        self.el_arr.append(el)
        self.arm_vel_arr.append(v_arm)
        self.lift_vel_arr.append(v_lift)
        self.rhoa_arr.append(rho_arm)
        self.rhol_arr.append(rho_lift)

        self.ref_lower_arr.append(ref_lower)
        self.ref_upper_arr.append(ref_upper)

        drive_msg = Twist()

        drive_msg.linear.x = cmd_vel
        drive_msg.angular.z = steer

        self.base_pub.publish(drive_msg)
    
        arm_speed = {'joint_arm_l0': v_arm, 'joint_arm_l1': v_arm, 'joint_arm_l2': v_arm, 'joint_arm_l3': v_arm}
        arm_speed['joint_lift'] = v_lift

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0.0).to_msg()

        point.velocities = [joint_velocity for joint_velocity in arm_speed.values()]
        joint_names = [key for key in arm_speed]

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        trajectory_goal.trajectory.joint_names = joint_names
        trajectory_goal.trajectory.points = [point]
        self.trajectory_client.send_goal_async(trajectory_goal)

        self.get_logger().info(f"cmd_vel={cmd_vel}, steer={steer}, vlift = {v_lift}, varm = {v_arm}")
        # self.get_logger().info(f"ed={ed}, eo={eo}")

        # self.get_logger().info(f"zdes: {z_des}, arm_des = {arm_des}")
        # self.get_logger().info(f"z = {z}, arm = {self.joint_states.position[3]}")

        # if t > 5:
        #     self.plot()

    def transform(self, x):
        a = 2
        return (1-np.exp(-(a*x)**2)) * np.tanh(a*x)

    def create_tube(self, start_set, goal_set, t_final):
        dt = 0.001
        self.time_arr = np.arange(0,t_final,dt)
        rho_l_x = np.array([start_set[0,0] + (goal_set[0,0] - start_set[0,0]) * np.tanh(t/(t_final - t)) for t in self.time_arr])
        rho_u_x = np.array([start_set[0,1] + (goal_set[0,1] - start_set[0,1]) * np.tanh(t/(t_final - t)) for t in self.time_arr])
        
        rho_l_y = np.array([start_set[1,0] + (goal_set[1,0] - start_set[1,0]) * np.tanh(t/(t_final - t)) for t in self.time_arr])
        rho_u_y = np.array([start_set[1,1] + (goal_set[1,1] - start_set[1,1]) * np.tanh(t/(t_final - t)) for t in self.time_arr])

        rho_l_lift = np.array([start_set[2,0] + (goal_set[2,0] - start_set[2,0]) * np.tanh(t/(t_final - t)) for t in self.time_arr])
        rho_u_lift = np.array([start_set[2,1] + (goal_set[2,1] - start_set[2,1]) * np.tanh(t/(t_final - t)) for t in self.time_arr])

        rho_l_arm = np.array([start_set[3,0] + (goal_set[3,0] - start_set[3,0]) * np.tanh(t/(t_final - t)) for t in self.time_arr])
        rho_u_arm = np.array([start_set[3,1] + (goal_set[3,1] - start_set[3,1]) * np.tanh(t/(t_final - t)) for t in self.time_arr])

        self.stt_val = np.array([self.time_arr, rho_l_x, rho_u_x, rho_l_y, rho_u_y, rho_l_lift, rho_u_lift, rho_l_arm, rho_u_arm])
        self.get_logger().info(f"{self.stt_val}")

        fig, axs = plt.subplots(2, 2, figsize=(12, 10))

        axs[0, 0].plot(self.time_arr, rho_l_x, label='x_l')
        axs[0, 0].plot(self.time_arr, rho_u_x, label='x_u')
        axs[0, 0].set_title('x vs time')
        axs[0, 0].set_xlabel('t')
        axs[0, 0].set_ylabel('x')
        axs[0, 0].legend()

        axs[0, 1].plot(self.time_arr, rho_l_y, label='y_l')
        axs[0, 1].plot(self.time_arr, rho_u_y, label='y_u')
        axs[0, 1].set_title('y vs time')
        axs[0, 1].set_xlabel('t')
        axs[0, 1].set_ylabel('y')
        axs[0, 1].legend()

        axs[1, 0].plot(self.time_arr, rho_l_lift, label='lift_l')
        axs[1, 0].plot(self.time_arr, rho_u_lift, label='lift_u')
        axs[1, 0].set_title('lift vs time')
        axs[1, 0].set_xlabel('t')
        axs[1, 0].set_ylabel('lift')
        axs[1, 0].legend()

        axs[1, 1].plot(self.time_arr, rho_l_arm, label='arm_l')
        axs[1, 1].plot(self.time_arr, rho_u_arm, label='arm_u')
        axs[1, 1].set_title('arm vs time')
        axs[1, 1].set_xlabel('t')
        axs[1, 1].set_ylabel('arm')
        axs[1, 1].legend()

        plt.tight_layout()
        plt.show()


    def get_ref_pose(self,t):
        gamL_arr = [self.stt_val[i,:] for i in range(1,8,2)]
        gamU_arr = [self.stt_val[i,:] for i in range(2,9,2)]
        
        # print(gamL_arr)
        # print(gamU_arr)

        time_arr = self.stt_val[0,:]

        tube_lower = np.array([np.interp(t, time_arr, x) for x in gamL_arr])
        tube_upper = np.array([np.interp(t, time_arr, x) for x in gamU_arr])

        return tube_lower, tube_upper
    
    def zero_vel(self):
        v_arm = 0.0
        v_lift = 0.0
        arm_speed = {'joint_arm_l0': v_arm, 'joint_arm_l1': v_arm, 'joint_arm_l2': v_arm, 'joint_arm_l3': v_arm}
        arm_speed['joint_lift'] = v_lift

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0.0).to_msg()

        point.velocities = [joint_velocity for joint_velocity in arm_speed.values()]
        joint_names = [key for key in arm_speed]

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        trajectory_goal.trajectory.joint_names = joint_names
        trajectory_goal.trajectory.points = [point]
        self.trajectory_client.send_goal_async(trajectory_goal)

    def plot(self):
        fig, axs = plt.subplots(3, 3, figsize=(12, 10))

        # Plot 1: Path of x and y, along with xref and yref
        axs[0, 0].plot(self.x_arr, self.y_arr, label='Path (x, y)', color='b')
        # axs[0, 0].plot((self.stt_val[1] + self.stt_val[2])/2, (self.stt_val[3] + self.stt_val[4])/2, label='Reference Path (xref, yref)', color='r', linestyle='--')
        self.ref_lower_arr = np.array(self.ref_lower_arr)
        self.ref_upper_arr = np.array(self.ref_upper_arr)

        axs[0, 0].plot(self.ref_lower_arr[:,0], self.ref_lower_arr[:,1], label = 'Tube Lower')
        axs[0, 0].plot(self.ref_upper_arr[:,0], self.ref_upper_arr[:,1], label = 'Tube Upper')

        axs[0, 0].set_title('Path: x vs y')
        axs[0, 0].set_xlabel('x')
        axs[0, 0].set_ylabel('y')
        axs[0, 0].legend()

        # Plot 2: ed vs time
        axs[0, 1].plot(self.ed_arr, label='ed (distance error)', color='g')
        axs[0, 1].plot(self.rhod_arr, label = 'funnel upper')
        axs[0, 1].plot(np.ones(len(self.ed_arr)) * self.rhod_lower, label = 'funnel lower')
        axs[0, 1].set_title('ed vs Time')
        axs[0, 1].set_xlabel('Time')
        axs[0, 1].set_ylabel('ed')
        axs[0, 1].legend()

        # Plot 3: eo vs time
        axs[1, 0].plot(self.eo_arr, label='eo (orientation error)', color='orange')
        axs[1, 0].plot(self.rhoo_arr, label = 'funnel upper')
        axs[1, 0].plot([-1*x for x in self.rhoo_arr], label = 'funnel lower')
        axs[1, 0].set_title('eo vs Time')
        axs[1, 0].set_xlabel('Time')
        axs[1, 0].set_ylabel('eo')
        axs[1, 0].legend()

        # Plot 4: cmd_vel vs time
        axs[1, 1].plot(self.cmd_vel_arr, label='cmd_vel (commanded velocity)', color='purple')
        axs[1, 1].set_title('cmd_vel vs Time')
        axs[1, 1].set_xlabel('Time')
        axs[1, 1].set_ylabel('cmd_vel')
        axs[1, 1].legend()

        # Plot 5: Steering angle vs time
        axs[2, 0].plot(self.steer_arr, label='Steering angle', color='brown')
        axs[2, 0].set_title('Steering Angle vs Time')
        axs[2, 0].set_xlabel('Time')
        axs[2, 0].set_ylabel('Steering angle (rad)')
        axs[2, 0].legend()

        axs[1, 2].plot(self.ea_arr, label='ea', color='orange')
        axs[1, 2].plot(self.rhoa_arr, label = 'funnel upper')
        axs[1, 2].plot([-1*x for x in self.rhoa_arr], label = 'funnel lower')
        axs[1, 2].set_title('ea vs Time')
        axs[1, 2].set_xlabel('Time')
        axs[1, 2].set_ylabel('ea')
        axs[1, 2].legend()

        axs[2, 2].plot(self.el_arr, label='el', color='orange')
        axs[2, 2].plot(self.rhol_arr, label = 'funnel upper')
        axs[2, 2].plot([-1*x for x in self.rhol_arr], label = 'funnel lower')
        axs[2, 2].set_title('el vs Time')
        axs[2, 2].set_xlabel('Time')
        axs[2, 2].set_ylabel('el')
        axs[2, 2].legend()

        axs[0, 2].plot(self.arm_vel_arr, label='arm_vel', color='purple')
        axs[0, 2].set_title('arm_vel vs Time')
        axs[0, 2].set_xlabel('Time')
        axs[0, 2].set_ylabel('arm_vel')
        axs[0, 2].legend()

        axs[2, 1].plot(self.lift_vel_arr, label='lift_vel', color='purple')
        axs[2, 1].set_title('lift_vel vs Time')
        axs[2, 1].set_xlabel('Time')
        axs[2, 1].set_ylabel('lift_vel')
        axs[2, 1].legend()
        
        plt.tight_layout()
        plt.show()
        

def main(args=None):
    # rclpy.init(args=args)
    node = DrivePublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
        node.zero_vel()
        node.plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
