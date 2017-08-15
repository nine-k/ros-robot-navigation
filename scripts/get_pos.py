#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf   

def quat_to_tuple(q):
    return (q.x, q.y, q.z, q.w)

def cb(data):
    rospy.loginfo(tf.transformations.euler_from_quaternion(quat_to_tuple(data.pose.pose.orientation))[2])


def Listener():
    rospy.init_node('Get_odom', anonymous=False)
    rospy.Subscriber('/odom', Odometry, cb, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    Listener()
