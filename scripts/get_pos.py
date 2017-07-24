#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def cb(data):
    rospy.loginfo(data.pose.pose.position)


def Listener():
    rospy.init_node('Get_odom', anonymous=False)
    rospy.Subscriber('/odom', Odometry, cb, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    Listener()
