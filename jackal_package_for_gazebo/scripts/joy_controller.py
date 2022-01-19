#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 19 22:02:03 2022
@author: mason
"""

''' import libraries '''
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)



''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.robot_vel_topic = rospy.get_param("/mobile_robot_vel_topic", "/jackal/jackal_velocity_controller/cmd_vel")
        self.mobile_robot_vel_pub = rospy.Publisher(self.robot_vel_topic, Twist, queue_size=3)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.rate = rospy.Rate(15)

        self.joy_check=0

        self.max_vel = 1.2
        self.yaw_rate = 1.0

    def joy_callback(self, msg):
        if len(msg.axes)>0 or len(msg.buttons)>0 :
            self.joy = msg
            self.joy_check=1

    def input(self):
        vel_input = Twist()
        vel_input.linear.x = self.joy.axes[1] * self.max_vel
        vel_input.angular.z = self.joy.axes[0] * self.yaw_rate
        self.mobile_robot_vel_pub.publish(vel_input)
##############################################################################################


''' main '''
if __name__ == '__main__':
    rbt_ctr = robot()
    while 1:
        try:
            if rbt_ctr.joy_check==1:
                rbt_ctr.input()
                rbt_ctr.rate.sleep()
            else: 
                rbt_ctr.rate.sleep()
                pass
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass
