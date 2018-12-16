#!/usr/bin/env python  
"""
Taken from https://github.com/BOTSlab/bupimo_src/blob/master/rvo2_laser/scripts/laser_max_min.py
Just prints the maximum and minimum ranges from the laser.
"""

import rospy
from math import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import math
import utility as util
import tf

def get_rotated_point(x,z):
    '''
    http://answers.ros.org/question/199215/transform-from-laser-frame-to-odom-frame-using-transformation-matrix-in-ros-c/
    '''
    laser_point = PointStamped()
    laser_point.header.frame_id = "body"
    #laser_point.header.stamp = ros::Time()
    laser_point.point.x = x
    laser_point.point.y = 0.
    laser_point.point.z = 0.

    odom_point = PointStamped()
    try:
        listener.transformPoint("body", laser_point, odom_point)
    except (tf.ransformException) as e:
        print "Received an exception trying to transform a point:",e


def scan_callback(scan):

    n = len(scan.ranges)
    points = []
    max_range = 0
    max_i = 0
    max_range_angle = None
    min_range = float('inf')
    min_range_angle = None
    min_i = 0
    frame_id = scan.header.frame_id
    '''
    try:
      (trans,rot) = listener.lookupTransform("/scanner", "/body", rospy.Time(1.0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
      print "exception",e
      return
    print trans, rot
    '''
    for i in range(0, n):
        rho = scan.ranges[i]
        theta = scan.angle_min + i * scan.angle_increment
        if not (rho == float('inf') or isnan(rho)):
            if rho >= max_range:
                max_i = i
                max_range = rho
                max_range_angle = theta
            if rho <= min_range:
                min_i = i
                min_range = rho
                min_range_angle = theta

    max_angle_degrees = util.to_degrees(max_range_angle)
    min_angle_degrees = util.to_degrees(min_range_angle)

    print "{:3.0f} max:{:.3f} at degrees:{:.1f}, {:3.0f} min:{:.3f} at degrees:{:.1f}".format(max_i, max_range, max_angle_degrees, min_i, min_range, min_angle_degrees)

if __name__ == '__main__':
    rospy.init_node('laser_max_min')

    #listener = tf.TransformListener()

    # Subscribe to scan
    rospy.Subscriber('scan', LaserScan, scan_callback)

rospy.spin()
