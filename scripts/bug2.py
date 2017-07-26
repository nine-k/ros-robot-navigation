#!/usr/bin/env python

import rospy
import math
from copy import deepcopy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

no_odom_reading = True #flag to tell if gazebo is up
ON_LINE_THRESH = 0.5 #threshold value to tell if robot is on the goal line

cur_pos = Point() #current position
cur_angle = 0 #current z axis euler angle

DEST = Point() #goal point
DEST.x = 5
DEST.y = 7

EPS = 0.2 #epsilon value

start_pos = Point() #first recived position value

pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10) #turtle bot speed topic

def sign(a): #sign function
    if (a > 0):
        return 1
    if (a < 0):
        return -1
    return 0

def write_speed(lin_v, ang_v=0): #function to assign a new speed to turtle bot
    msg = Twist()
    msg.linear.x = lin_v
    msg.angular.z = ang_v
    pub.publish(msg)

def quat_to_tuple(q): #function to make a tuple out of a geometry_msgs/Quaternion
    return (q.x, q.y, q.z, q.w)

def update_position(data): #callback function for the odometry topic
    global cur_pos, no_odom_reading, cur_angle, start_pos
    cur_angle = euler_from_quaternion(quat_to_tuple(data.pose.pose.orientation))[2] #translate quaternion to euler angel and take only the z axis value
    cur_pos = data.pose.pose.position
    if no_odom_reading:
        start_pos = deepcopy(cur_pos)
    no_odom_reading = False

def is_on_goal_line(): #function to check if turtle bot is on goal line
    #if (cur_pos, start_pos) and (cur_pos, DEST) are collinear robot is considered to be on the goal line
    coeff_x = (cur_pos.x - start_pos.x) / (DEST.x - start_pos.x)
    coeff_y = (cur_pos.y - start_pos.y) / (DEST.y - start_pos.y)
    return abs(coeff_x - coeff_y) < ON_LINE_THRESH

def vec_len(p1, p2): #calculate length of a vector
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def calc_angle_to_goal():
    dist = vec_len(DEST, cur_pos)
    phi = math.acos((DEST.x - cur_pos.x) / dist) #calculate angle between (DEST, cur_pos) and the x axis
    if (DEST.y - cur_pos.y) < 0: #convert angle between vectors to euler angle
        phi *= -1
    return phi

TURN_K = -0.2 #angular speed proportional coefficient when heading towards the goal

def go_to_goal():
    angle_to_goal = calc_angle_to_goal()
    #rospy.loginfo('angle to goal ' + str(angle_to_goal * 180 / math.pi))
    #rospy.loginfo('cur_angle ' + str(cur_angle * 180 / math.pi))
    turn_angle = cur_angle - angle_to_goal #difference between the current oriention of the robot and the desired orientation
    if turn_angle < -math.pi: #calcualte minimum turn angle
        turn_angle += 2 * math.pi
    if turn_angle > math.pi:
        turn_angle -= 2 * math.pi
    if abs(turn_angle) > math.pi / 6: #if the turn angle is to big first rotate in one point to avoid wide turn arc
        rospy.loginfo('turning_to_dest')
        write_speed(0, -sign(turn_angle) * 0.3)
    else:
        rospy.loginfo('going_to_dest')
        write_speed(0.15, turn_angle * TURN_K) #angular speed is proportional to the turn angle

def routine():
    dist_to_goal = vec_len(cur_pos, DEST) #calculate distance to goal point
    rospy.loginfo('dist to goal ' + str(dist_to_goal))
    if (dist_to_goal < 0.2): #check if robot is at goal
        rospy.loginfo('got_to_goal')
        exit()
    if is_on_goal_line():
        go_to_goal()


def Listener():
    rospy.init_node('BUG2', anonymous=False) #initialize a ROS node called BUG2
    rospy.Subscriber('/odom', Odometry, update_position, queue_size=1) #subscribe to odometry topic
    while not rospy.is_shutdown():
        if not no_odom_reading:
            routine()


if __name__ == '__main__':
    Listener()
