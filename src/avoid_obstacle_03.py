#!/usr/bin/env python
import rospy
import yaml
import numpy as np
import math
import sys

from RosInterface import ROSInterface

from geometry_msgs.msg import Twist
from sensor_msgs.msg   import LaserScan

# Ideas taken from https://github.com/Arkapravo/laser_obstacle_avoidance_pr2/blob/master/src/obstacle_avoidance_pr2.cpp


class RobotControl(object):
    """
    Robot interface class. In charge of getting sensor measurements (ROS subscribers),
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map, occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        """

        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

    def to_degrees(self,rad):
	angle_degrees = rad*180/math.pi
	return angle_degrees

    def is_valid_distance(self,d):
        """ 
        Discard Inf distances
        """
	if (np.isnan(d) or np.isinf(d)):
		return False
	else:
		return True


    def is_within_range(self,i):
        """ 
        Only consider -90 to 90 degrees
        """
	#return -math.pi/4 < i < math.pi/4 
	return math.pi/2 < abs(i) # LIDAR rotated backwards

    def process_measurements(self):
        """ 
        Basic code to run autonomously
        """
	# Check samples from https://github.com/Arkapravo/laser_obstacle_avoidance_pr2/blob/master/src/obstacle_avoidance_pr2.cpp

        scan = self.ros_interface.get_scan()

        angular_z = 0
        linear_x = 0
        k = 0.4 # 0.3
        min_linear = 0.09
        rotate_fast_value = 1. # 5.8 # rad/s
        linear_k = 0.15
        go_forward = 0.5 # default value
        action = "None"
        if(scan == None):
            return

        #print "i:%.5f"%angle+"(%.5f"%self.to_degrees(angle)+"degrees) x:%.5f"%real_dist+" x:%.2f"%linear+" z:%.2f"%rotational

        min_range = scan.ranges[0]
	min_range_angle = 0
	min_angle =0
	for j in range(0, 360): #increment by one degree
            real_dist = scan.ranges[j]
            angle = scan.angle_min + j * scan.angle_increment
            if self.is_within_range(angle) and self.is_valid_distance(real_dist):
	        if (real_dist < min_range):
	            min_range = real_dist
	            min_angle = angle
	            min_range_angle = j/2

	#print "minimum range is [%.3f]"%min_range+" at an angle of [%.3f]"%min_range_angle
	if (min_range <= 0.5):  # min_range<=0.5 gave box pushing like behaviour, min_range<=1.2 gave obstacle avoidance
	    if (min_range_angle < 90):
	        angular_z = rotate_fast_value
	        linear_x = 0
                action = "left "
	    else:
	        angular_z = -rotate_fast_value;
	        linear_x = 0;
                action = "right"
	else:
	    angular_z = 0
	    linear_x = go_forward
            action = "straight"
	
	print "action:"+action+" min distance[%.2f]"%min_range+" angle [%.2f]"%min_range_angle+" (%.1f)"%angle+" min angle %.1f"%min_angle

        self.ros_interface.command_velocity(linear_x, angular_z)
        return
    
def main(args):
    rospy.init_node('avoid_obstacle_03')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']
    rospy.loginfo("[robot_control]: ready")

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body)

    r = rospy.Rate(10) # original 60Hz
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass

