#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon June 28 02:28:30 2021

@author: mason
"""

''' import libraries '''
import rospy

from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''

class viz():
    def __init__(self):
        rospy.init_node('viz', anonymous=True)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gt_cb)
        rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.encoder_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.vio_cb)
        rospy.Subscriber("/estimated_pose_diff", PoseStamped, self.estimate_cb)

        self.drone_gt = rospy.Publisher("drone_gt", PointStamped, queue_size=2)
        self.mobile_gt = rospy.Publisher("mobile_gt", PointStamped, queue_size=2)
        self.encoder = rospy.Publisher("encoder", PointStamped, queue_size=2)
        self.vio = rospy.Publisher("vio", PointStamped, queue_size=2)
        self.estimated_drone = rospy.Publisher("estimated_drone", PointStamped, queue_size=2)
        self.estimated_mobile = rospy.Publisher("estimated_mobile", PointStamped, queue_size=2)
        self.estimated_drone_path_pub = rospy.Publisher("estimated_drone_path", Path, queue_size=2)
        self.estimated_mobile_path_pub = rospy.Publisher("estimated_mobile_path", Path, queue_size=2)

        self.gt_check = False
        self.gt_check2 = False
        self.estimated_check=False
        self.rate=rospy.Rate(5)

    def gt_cb(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i]=="iris0":
                self.drone = msg.pose[i]
                m = PointStamped()
                m.header.stamp = rospy.Time.now()
                m.header.frame_id = "map"
                m.point.x = self.drone.position.x
                m.point.y = self.drone.position.y
                m.point.z = self.drone.position.z
                self.drone_gt.publish(m)
                self.gt_check= True
            if msg.name[i]=="jackal1":
                self.mobile = msg.pose[i]
                m = PointStamped()
                m.header.stamp = rospy.Time.now()
                m.header.frame_id = "map"
                m.point.x = self.mobile.position.x
                m.point.y = self.mobile.position.y
                m.point.z = self.mobile.position.z
                self.mobile_gt.publish(m)
                self.gt_check2= True

    def encoder_cb(self, msg):
        d=msg.pose.pose.position
        m = PointStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.point.x = d.x
        m.point.y = d.y
        m.point.z = d.z
        self.encoder.publish(m)

    def vio_cb(self, msg):
        d=msg.pose.position
        m = PointStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.point.x = d.x
        m.point.y = d.y
        m.point.z = d.z
        self.vio.publish(m)

    def estimate_cb(self, msg):
        if (self.gt_check2 and self.gt_check):
            d=msg.pose.position
            #d: mobile-drone
            m = PointStamped()
            m2 = PointStamped()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "map"
            m.point.x = d.x + self.drone.position.x
            m.point.y = d.y + self.drone.position.y
            m.point.z = d.z + self.drone.position.z
            m2.header.stamp = rospy.Time.now()
            m2.header.frame_id = "map"
            m2.point.x = self.mobile.position.x - d.x
            m2.point.y = self.mobile.position.y - d.y
            m2.point.z = self.mobile.position.z - d.z

            self.mobile_pose = PoseStamped()
            self.drone_pose = PoseStamped()
            self.mobile_pose.header.stamp = rospy.Time.now()
            self.mobile_pose.header.frame_id = "map"
            self.mobile_pose.pose.position = m.point
            self.mobile_pose.pose.orientation.w = 1
            self.drone_pose.header.stamp = rospy.Time.now()
            self.drone_pose.header.frame_id = "map"
            self.drone_pose.pose.position = m2.point
            self.drone_pose.pose.orientation.w = 1

            self.estimated_check=True
            self.estimated_mobile.publish(m)
            self.estimated_drone.publish(m2)

''' main '''
pub_class = viz()
estimated_drone_path = Path()
estimated_mobile_path = Path()
estimated_drone_path.header.frame_id = "map"
estimated_mobile_path.header.frame_id = "map"

if __name__ == '__main__':
    while 1:
        try:
            if (pub_class.gt_check2 and pub_class.gt_check and pub_class.estimated_check):
                estimated_drone_path.header.stamp = rospy.Time.now()
                estimated_drone_path.poses.append(pub_class.drone_pose)
                estimated_mobile_path.header.stamp = rospy.Time.now()
                estimated_mobile_path.poses.append(pub_class.mobile_pose)
                pub_class.estimated_drone_path_pub.publish(estimated_drone_path)
                pub_class.estimated_mobile_path_pub.publish(estimated_mobile_path)
            pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass
