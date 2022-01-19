#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue May 19 00:28:30 2020

@author: mason
"""

''' import libraries '''
import time
import numpy as np

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''

class path_pub():
    def __init__(self):
        rospy.init_node('gt_path_pubb', anonymous=True)
        self.parent_frame_id = rospy.get_param("/parent_frame_id", 'map')
        self.append_rate = rospy.get_param("/append_rate", 3)
        self.robot_name = "/"
        self.target_name = "jackal"

        self.gt_path_pub = rospy.Publisher("gt_robot_path", Path, queue_size=2)
        self.gt_path_pub2 = rospy.Publisher("gt_target_path", Path, queue_size=2)
        self.gt_poses = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gtcallback)

        self.robot_path = Path()
        self.target_path = Path()
        self.robot_check = 0
        self.target_check = 0

        self.rate = rospy.Rate(self.append_rate)

    def gtcallback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i]==self.robot_name:
                self.robot_pose = msg.pose[i]
                self.robot_check= 1
            elif msg.name[i]==self.target_name:
                self.target_pose = msg.pose[i]
                self.target_check= 1

''' main '''
path_pub_ = path_pub()

if __name__ == '__main__':
    while 1:
        try:
            if path_pub_.robot_check == 1:
                pose = PoseStamped()
                pose.pose = path_pub_.robot_pose
                pose.header.frame_id = path_pub_.parent_frame_id
                pose.header.stamp = rospy.Time.now()
                path_pub_.robot_path.poses.append(pose)
                path_pub_.robot_path.header.frame_id = path_pub_.parent_frame_id 
                path_pub_.robot_path.header.stamp = rospy.Time.now()
                path_pub_.gt_path_pub.publish(path_pub_.robot_path)
            if path_pub_.target_check == 1:
                pose = PoseStamped()
                pose.pose = path_pub_.target_pose
                pose.header.frame_id = path_pub_.parent_frame_id
                pose.header.stamp = rospy.Time.now()
                path_pub_.target_path.poses.append(pose)
                path_pub_.target_path.header.frame_id = path_pub_.parent_frame_id 
                path_pub_.target_path.header.stamp = rospy.Time.now()
                path_pub_.gt_path_pub2.publish(path_pub_.target_path)
            path_pub_.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass