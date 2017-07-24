#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

current_position = Point()

def update_position(data):
    cur_pos = data.pose.pose.position
    rospy.loginfo(cur_pos)


def Listener():
    rospy.init_node('Get_odom', anonymous=False)
    rospy.Subscriber('/odom', Odometry, update_position, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    Listener()
