#!/usr/bin/env python

import rospy
import math
from copy import deepcopy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

first_odom_reading = True
ON_LINE_THRESH = 0.5

cur_pos = Point()
cur_angle = 0

DEST = Point()
DEST.x = 5
DEST.y = 7

EPS = 0.2

start_pos = Point()

pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

def write_speed(lin_v, ang_v=0):
    msg = Twist()
    msg.linear.x = lin_v
    msg.angular.z = ang_v
    pub.publish(msg)

def quat_to_tuple(q):
    return (q.x, q.y, q.z, q.w)

def update_position(data):
    global cur_pos, first_odom_reading
    cur_angle = euler_from_quaternion(quat_to_tuple(data.pose.pose.orientation))
    cur_pos = data.pose.pose.position
    if first_odom_reading:
        start_pos = deepcopy(cur_pos)
    first_odom_reading = False
    rospy.loginfo(cur_pos)

def is_on_goal_line():
    coeff_x = (cur_pos.x - start_pos.x) / (DEST.x - start_pos.x)
    coeff_y = (cur_pos.y - start_pos.y) / (DEST.y - start_pos.y)
    return abs(coeff_x - coeff_y) < ON_LINE_THRESH

def go_to_goal():

def routine():
    if is_on_goal_line():
        go_to_goal()

def Listener():
    rospy.init_node('Get_odom', anonymous=False)
    rospy.Subscriber('/odom', Odometry, update_position, queue_size=1)
    while not rospy.is_shutdown():
        if not first_odom_reading:
            routine()


if __name__ == '__main__':
    Listener()
