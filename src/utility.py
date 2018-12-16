#!/usr/bin/env python
"""
Utility functions that are commonly used across all programs
"""
import numpy as np
import math

import roslib
from std_msgs.msg import (
    Header,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Quaternion,
)
from tf.transformations import *

min_rotation   = math.pi/180  # 1 degree
max_rotation   = min_rotation * 20
max_distance   = 99999

def get_t_R(pose):
    """
    Returns the translation vector (4x1) and rotation matrix (4x4) from a pose message
    """
    t = np.transpose(np.matrix([pose.position.x,pose.position.y,pose.position.z,0]))
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R = quaternion_matrix(quat)
    #R = R_full
    return t,R

def make_pose_stamped_msg(t,R):
    """
    Returns a pose stamped message from a translation vector and rotation matrix (4x4) for publishing.
    NOTE: Does not set the target frame.
    """
    pose_stamped_msg=PoseStamped()
    pose_stamped_msg.header=Header()
    pose_stamped_msg.header.stamp=rospy.Time.now()
    pose_msg=Pose()
    pose_msg.position.x=t[0]
    pose_msg.position.y=t[1]
    pose_msg.position.z=t[2]
    quat=quaternion_from_matrix(R)
    pose_msg.orientation.x=quat[0]
    pose_msg.orientation.y=quat[1]
    pose_msg.orientation.z=quat[2]
    pose_msg.orientation.w=quat[3]
    pose_stamped_msg.pose=pose_msg
    return pose_stamped_msg

def to_degrees(rad):
    angle_degrees = rad*180/math.pi
    return angle_degrees

def is_valid_distance(d):
    """ 
    Discard Inf distances
    """
    if (np.isnan(d) or np.isinf(d)):
        return False
    else:
        return True

def is_within_range(i):
    """ 
    Only consider -90 to 90 degrees
    """
    return math.pi/2 < abs(i) # LIDAR rotated backwards

def is_forward_right(i):
    """ 
    Only consider 90 to 180 degrees
    """
    #return i <= math.pi/2
    return math.pi/2 <= i <= math.pi # LIDAR rotated backwards

def is_forward_left(i):
    """ 
    Only consider -90 to -180 degrees
    """
    #return i > math.pi/2
    return -math.pi <= i <= -math.pi/2 # LIDAR rotated backwards

def is_backwards_right(i):
    """ 
    Only consider 0 to 90 degrees
    """
    return 0 < i < math.pi/2 # LIDAR rotated backwards

def is_backwards_left(i):
    """ 
    Only consider 0 to -90 degrees
    """
    return -math.pi/2 < i < 0 # LIDAR rotated backwards

def proportional_rotation(angle):
    return (max_rotation - min_rotation)*angle/(math.pi/2) + min_rotation


