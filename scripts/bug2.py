#!/usr/bin/env python

import rospy
import math
from copy import deepcopy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

no_odom_reading = True
ON_LINE_THRESH = 0.5

cur_pos = Point()
cur_angle = 0

DEST = Point()
DEST.x = 5
DEST.y = 7

EPS = 0.2

start_pos = Point()

pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

def sign(a):
    if (a > 0):
        return 1
    if (a < 0):
        return -1
    return 0

def write_speed(lin_v, ang_v=0):
    msg = Twist()
    msg.linear.x = lin_v
    msg.angular.z = ang_v
    pub.publish(msg)

def quat_to_tuple(q):
    return (q.x, q.y, q.z, q.w)

def update_position(data):
    global cur_pos, no_odom_reading, cur_angle, start_pos
    cur_angle = euler_from_quaternion(quat_to_tuple(data.pose.pose.orientation))[2]
    cur_pos = data.pose.pose.position
    if no_odom_reading:
        start_pos = deepcopy(cur_pos)
    no_odom_reading = False
    rospy.loginfo(cur_pos)

def is_on_goal_line():
    coeff_x = (cur_pos.x - start_pos.x) / (DEST.x - start_pos.x)
    coeff_y = (cur_pos.y - start_pos.y) / (DEST.y - start_pos.y)
    return abs(coeff_x - coeff_y) < ON_LINE_THRESH

def vec_len(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def calc_angle_to_goal():
    dist = vec_len(DEST, cur_pos)
    phi = math.acos((DEST.x - cur_pos.x) / dist)
    if (DEST.y - cur_pos.y) < 0:
        phi *= -1
    return phi

TURN_K = -0.2

def go_to_goal():
    angle_to_goal = calc_angle_to_goal()
    #rospy.loginfo('angle to goal ' + str(angle_to_goal * 180 / math.pi))
    #rospy.loginfo('cur_angle ' + str(cur_angle * 180 / math.pi))
    turn_angle = cur_angle - angle_to_goal
    if turn_angle < -math.pi:
        turn_angle += 2 * math.pi
    if turn_angle > math.pi:
        turn_angle -= 2 * math.pi
    if abs(turn_angle) > math.pi / 6:
        rospy.loginfo('turning_to_dest')
        write_speed(0, -sign(turn_angle) * 0.3)
    else:
        rospy.loginfo('going_to_dest')
        write_speed(0.15, turn_angle * TURN_K)

def routine():
    dist_to_goal = vec_len(cur_pos, DEST)
    rospy.loginfo('dist to goal ' + str(dist_to_goal))
    if (dist_to_goal < 0.2):
        rospy.loginfo('got_to_goal')
        exit()
    if is_on_goal_line():
        go_to_goal()


def Listener():
    rospy.init_node('Get_odom', anonymous=False)
    rospy.Subscriber('/odom', Odometry, update_position, queue_size=1)
    while not rospy.is_shutdown():
        if not no_odom_reading:
            routine()


if __name__ == '__main__':
    Listener()
