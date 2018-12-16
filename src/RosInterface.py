# -*- coding: utf-8 -*-
#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan

from std_msgs.msg import (
    Header,
)

from sensor_msgs.msg import Imu

from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    Pose,
    Twist,
)

import cv2
import yaml
import numpy as np

import sys

class ROSInterface(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, t_cam_to_body):
        """
        Initialize the class
        """
        # Internal variables
        self._no_sonar = True
        self._no_lidar = True
        self._no_imu = True
        self._imu = None
        self._range = None
        self._t = None
        self._R = None
        self._R_cam2bot = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
        self._t_cam2bot = t_cam_to_body
        self._R_tag2bot = np.array([[0,-1,0,0],[0,0,1,0],[-1,0,0,0],[0,0,0,1]])

        # ROS publishers and subscribers
        self._pub = rospy.Publisher("/cmd_vel", Twist,queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self._lidar_callback)
        rospy.Subscriber("/imu", Imu, self._imu_callback)

    def _lidar_callback(self,r):
        """
        Callback function for LIDAR measurements
        """
        if (r is None): # or r.range is None or r.range == 0 or math.isnan(r.range)
            return

        self._lidar = r
        self._no_lidar = False

    def _sonar_callback(self,r):
        """
        Callback function for Sonar measurements
        """
        if (r is None or r.range is None or r.range == 0 or math.isnan(r.range)):
            return

        self._range = r.range
        self._no_sonar = False

    def _imu_callback(self, imu):
        """
        Callback function for IMU measurements
        """
        self._imu = np.array([[imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, imu.angular_velocity.z, imu.header.stamp.to_sec()]]).T
        self._no_imu = False

    def get_imu(self):
        if self._no_imu:
            return None
        self._no_imu = True
        return self._imu
        
    def get_scan(self):
        """
        Returns information about the last measurement seen from the LIDAR if any.
        """
        if self._no_lidar:
            return None
        self._no_lidar = True
        return self._lidar

    def get_range(self):
        """
        Returns information about the last measurement seen if any.
        Returns None if no new measurement is retrieved.
        """
        if self._no_lidar:
            return None
        self._no_lidar = True
        return self._range

    def command_velocity(self,vx,wz):
        """
        Commands the robot to move with linear velocity vx and angular velocity wz
        """
        twist=Twist()
        twist.linear.x = vx
        twist.angular.z = wz
        self._pub.publish(twist)
