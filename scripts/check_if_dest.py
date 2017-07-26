#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

cur_pos = Point()

DEST = Point()
DEST.x = 5
DEST.y = 7

EPS = 0.4

def update_position(data):
    global cur_pos
    cur_pos = data.pose.pose.position
    rospy.loginfo(cur_pos)


def routine():
    dist_x = abs(cur_pos.x - DEST.x)
    dist_y = abs(cur_pos.y - DEST.y)
#    rospy.loginfo(str(dist_x) + ' ' + str(dist_y))
    if dist_x < EPS and dist_y < EPS:
        rospy.loginfo("GOT THERE!")

def Listener():
    rospy.init_node('Get_odom', anonymous=False)
    rospy.Subscriber('/odom', Odometry, update_position, queue_size=1)
    while not rospy.is_shutdown():
        routine()


if __name__ == '__main__':
    Listener()
