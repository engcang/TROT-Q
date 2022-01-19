#!/usr/bin/env python

import numpy as np
from scipy.optimize import minimize
from math import pow, sqrt, cos, sin

import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import time
import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def distance(x1, y1, x2, y2):
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2))

def rpy_saturation(angle):
    if angle>np.pi:
        angle=angle-2*np.pi
    if angle<-np.pi:
        angle=angle+2*np.pi
    return angle


class mpc_ctrl():
    def __init__(self):

        ### ROS Things
        rospy.init_node('mpc_controlelr', anonymous=True)
        self.pose_sub = rospy.Subscriber('/Odometry', Odometry, self.pose_cb)
        self.traj_sub = rospy.Subscriber('/best_path', Path, self.traj_cb)
        self.control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.new_path_pub = rospy.Publisher('/new_path', Bool, queue_size=2)
        self.pose_in=False
        self.traj_in=False
        self.rate = rospy.Rate(5)
        self.new_path_msg = Bool()
        self.new_path_msg.data = True

        ### MPC setup
        self.horizon = 7
        self.dt = 0.1
        self.num_inputs = 3 # linear_x, linear_y, angular_z (v, s, w)

        self.v_min = 0.0
        self.v_max = 0.5
        self.s_min = -0.12
        self.s_max = 0.12
        self.w_min = -0.7
        self.w_max = 0.7
        self.bounds = []
        for i in range(self.horizon):
            self.bounds += [[self.v_min, self.v_max]]
            self.bounds += [[self.s_min, self.s_max]]
            self.bounds += [[self.w_min, self.w_max]]
        self.bounds = np.array(self.bounds)

        self.u = np.zeros(self.horizon*self.num_inputs)

        self.position_weight = 1.0
        self.yaw_weight = 1.0
        self.input_weight = 0.5
        self.input_smoothness_weight = 2.0


    def pose_cb(self, msg):
        current_pose = msg.pose.pose.position #x, y, z
        _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.pose_in=True

        ### mpc start
        tic = time.time()
        if self.traj_in:
            current_state = np.array([current_pose.x, current_pose.y, rpy_saturation(yaw)])

            if distance(current_state[0], current_state[1], self.traj_ref[0], self.traj_ref[1]) < 0.8:
                self.new_path_pub.publish(self.new_path_msg)

            for i in range(self.num_inputs):
                self.u = np.delete(self.u, 0) #remove first input
                self.u = np.append(self.u, self.u[-self.num_inputs]) #copy last input

            u_solution = minimize(self.cost_function, self.u, (current_state, self.traj_ref),
                                  method='SLSQP', bounds=self.bounds, tol=1e-4, options = {'disp': False}) #disp - debugging
            self.u = u_solution.x
            # print(u_solution.x) #solution is stored in "x"
            # print(u_solution.success)
            # print(u_solution.message)

            solved_input = Twist()
            solved_input.linear.x = u_solution.x[0]
            solved_input.linear.y = u_solution.x[1]
            solved_input.angular.z = u_solution.x[2]
            self.control_pub.publish(solved_input)
            toc = time.time()
            print('MPC time spent: %.5f, solved: %s' %(toc - tic, u_solution.success))
        return

    def traj_cb(self, msg):
        if len(msg.poses) > 1 :
            last_idx=len(msg.poses)-1
            ref_x = msg.poses[last_idx].pose.position.x
            ref_y = msg.poses[last_idx].pose.position.y
            _, _, ref_yaw = euler_from_quaternion([msg.poses[last_idx].pose.orientation.x, msg.poses[last_idx].pose.orientation.y, msg.poses[last_idx].pose.orientation.z, msg.poses[last_idx].pose.orientation.w])
            self.traj_ref = np.array([ref_x, ref_y, rpy_saturation(ref_yaw)])
            self.traj_in=True
        return

    def cost_function(self,u, *args):
        curr_state = args[0]
        ref = args[1]
        cost = 0.0
        for i in range(self.horizon):
            prev_state = curr_state
            curr_state = self.plant(prev_state, self.dt, u[i*self.num_inputs], u[i*self.num_inputs+1], u[i*self.num_inputs+2])

            #tracking cost
            cost += self.position_weight * pow(curr_state[0]-ref[0], 2)
            cost += self.position_weight * pow(curr_state[1]-ref[1], 2)
            cost += self.yaw_weight * pow(rpy_saturation(curr_state[2]-ref[2]), 2)

            #input cost
            cost += self.input_weight * pow(u[i*self.num_inputs], 2)
            cost += self.input_weight * pow(u[i*self.num_inputs+1], 2)
            cost += self.input_weight * pow(u[i*self.num_inputs+2], 2)

            #input smoothness
            if i>0:
                cost += self.input_smoothness_weight * pow(u[i*self.num_inputs]-u[(i-1)*self.num_inputs], 2)
                cost += self.input_smoothness_weight * pow(u[i*self.num_inputs+1]-u[(i-1)*self.num_inputs+1], 2)
                cost += self.input_smoothness_weight * pow(u[i*self.num_inputs+2]-u[(i-1)*self.num_inputs+2], 2)  
        return cost


    def plant(self, prev_state, dt, v, s, w):
        x_t = prev_state[0]
        y_t = prev_state[1]
        yaw_t = prev_state[2]

        x_t_1 = x_t + (v*cos(yaw_t) - s*sin(yaw_t))*dt
        y_t_1 = y_t + (v*sin(yaw_t) + s*cos(yaw_t))*dt
        yaw_t_1 = rpy_saturation(yaw_t + w*dt)
        return [x_t_1, y_t_1, yaw_t_1]



if __name__ == '__main__':
    mpc_ctrl_ = mpc_ctrl()
    time.sleep(0.5)
    ### Init
    mpc_ctrl_.new_path_pub.publish(mpc_ctrl_.new_path_msg)
    while 1:
        try:
            mpc_ctrl_.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)