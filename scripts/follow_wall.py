#!/usr/bin/env python

import rospy
import math
from copy import deepcopy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

avoiding_obstacle = False
no_lidar_readings = True
new_lidar_reading = False
following_wall = False
no_odom_readings = True #flag to tell if gazebo is up
ON_LINE_THRESH = 0.2 #threshold value to tell if robot is on the goal line
OBSTACLE_THRESH = 1.7

cur_pos = Point() #current position
cur_angle = 0 #current z axis euler angle

DEST = Point() #goal point
DEST.x = 5
DEST.y = 7

min_dist_to_goal = 10000000000.

EPS = 0.2 #epsilon value

lidar_readings = [0] * 640

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
    global cur_pos, no_odom_readings, cur_angle, start_pos
    cur_angle = euler_from_quaternion(quat_to_tuple(data.pose.pose.orientation))[2] #translate quaternion to euler angel and take only the z axis value
    cur_pos = data.pose.pose.position
    if no_odom_readings:
        start_pos = deepcopy(cur_pos)
    no_odom_readings = False

def update_lidar(data):
    global lidar_readings, no_lidar_readings, new_lidar_reading
    no_lidar_readings = False
    new_lidar_reading = True
    #rospy.loginfo('got lidar ' + str(data.ranges[0]))
    lidar_readings = data.ranges

TARGET_DIST = 1.7
PID_SPEED = 0.3
MAX_DIST = 10.0
MIN_DIST = 0.45
dT = 0.033
RANGES_SIZE = 640

K_p = 0.4
K_i = 0.1
K_d = 0.06

err_int = 0
prev_err = 0
prev_dist = 1

def PID(cur_val, dest_val):
    #rospy.loginfo('current distance to wall: ' + str(cur_val))
    global err_int
    global prev_err

    err = dest_val - cur_val

    err_diff = (err - prev_err) / dT
    prev_err = err

    err_int += err * dT

    return K_p * err + K_i * err_int + K_d * err_diff

def fuzzy_follow(dist, t_dist):
    if (dist > t_dist + 0.2):
        write_speed(0.1, -0.1)
    elif (dist > t_dist - 0.2):
        write_speed(0.2)
    else:
        write_speed(0.1, 0.1)

def lost_wall():
    return math.isnan(lidar_readings[0])

def wall_infront():
    return not math.isnan(lidar_readings[319]) and lidar_readings[319] < 1.2

def turn_90_deg():
    while math.isnan(lidar_readings[0]):
        write_speed(0.2, -0.1)

def find_wall():
    write_speed(0.5)
    rospy.sleep(0.8)
    while (math.isnan(lidar_readings[0])):
        write_speed(0.1, -.1)

def follow_wall():
    global new_lidar_reading, following_wall
    if new_lidar_reading:
        operation_type = ''
        if wall_infront():
            operation_type = 'dead end'
            write_speed(0, 0.2)
        elif lost_wall():
            operation_type = 'lost wall'
            find_wall()
        else:
            operation_type = 'PID'
            write_speed(PID_SPEED, PID(lidar_readings[0], TARGET_DIST))
            #fuzzy_follow(lidar_readings[0], TARGET_DIST)
        rospy.loginfo(operation_type)
        new_lidar_reading = False

def routine():  
    follow_wall()


def Listener():
    rospy.init_node('BUG2', anonymous=False) #initialize a ROS node called BUG2
    global K_p, K_i, K_d
    K_p = rospy.get_param('/gains/Kp', K_p)
    K_i = rospy.get_param('/gains/Ki', K_i)
    K_d = rospy.get_param('/gains/Kd', K_d)
    rospy.loginfo('Kp: ' + str(K_p) + ' Ki: ' + str(K_i) + ' Kd: ' + str(K_d))
    rospy.Subscriber('/odom', Odometry, update_position, queue_size=1) #subscribe to odometry topic
    rospy.Subscriber('/scan', LaserScan, update_lidar, queue_size=1)
    while not rospy.is_shutdown():
        if not no_odom_readings and not no_lidar_readings:
            routine()


if __name__ == '__main__':
    Listener()
