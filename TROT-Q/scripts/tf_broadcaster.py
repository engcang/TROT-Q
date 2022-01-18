#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import tf

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class caster():
    def __init__(self):
        rospy.init_node('tf_broadcaster', anonymous=True)
        self.camera_link_name = rospy.get_param("/depth_base", 'd455/camera_link')
        self.lidar_link_name = rospy.get_param("/pcl_base", 'os0_lidar')
        self.body_link_name = rospy.get_param("/body_base", 'base_body')
        self.pose_sub = rospy.Subscriber('/Odometry', Odometry, self.base_cb)
        self.rate = rospy.Rate(5)

        self.br = tf.TransformBroadcaster()

    def base_cb(self, msg):
        stamp_  = msg.header.stamp
        self.br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),\
(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),\
stamp_ ,self.body_link_name,"map")

        self.br.sendTransform((0.3, 0.0, 0.15), (0.5,-0.5,0.5,-0.5), stamp_ , self.camera_link_name, self.body_link_name)
        self.br.sendTransform((0.0, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0), stamp_ , self.lidar_link_name, self.body_link_name)

        return

if __name__ == '__main__':
    cas = caster()
    while 1:
        try:
            cas.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)