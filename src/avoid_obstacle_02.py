#!/usr/bin/env python
import rospy
import yaml
import numpy as np
import math
import sys

from RosInterface import ROSInterface

from geometry_msgs.msg import Twist
from sensor_msgs.msg   import LaserScan

class RobotControl(object):
    """
    Robot interface class. In charge of getting sensor measurements (ROS subscribers),
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body, min_lin_x, max_lin_x, rotate_fast_value, linear_k, k, min_linear_vel):
        """
        Initialize the class
        """
        self.min_lin_x = min_lin_x
        self.max_lin_x = max_lin_x
        self.rotate_fast_value = rotate_fast_value
        self.linear_k = linear_k
        self.k = k
        self.min_linear_vel = min_linear_vel

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
	# Check samples from https://github.com/markwsilliman/turtlebot/blob/master/goforward_and_avoid_obstacle.py

        scan = self.ros_interface.get_scan()

        linear = 0
        rotational = 0
        rcount = 0
        hack = False
        msg = ""

        if(scan == None):
            return

        rcount = 0
        for i, real_dist in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            denom = 1. + real_dist * real_dist
            if self.is_within_range(angle) and self.is_valid_distance(real_dist):
                linear += math.sin(angle) / denom * 1.
                rotational += math.cos(angle) / denom * 1.
                rcount += 1
                #print "%.5f"%self.to_degrees(angle)+"degrees x:%.5f"%real_dist+" x:%.2f"%linear+" z:%.2f"%rotational

        try:
          linear /= rcount
          rotational /= rcount
        except Exception as e:
          pass

        """
	if (linear > self.k):
        	linear = self.k
        elif (linear < -self.k):
        	linear = -self.k
        """

        linear_x = linear
        angular_z = rotational

        linear_x = linear # check this
	
	print "linear_x:%.5f"%linear_x+" angular_z:%.1f"%self.to_degrees(angular_z)+"(%.2f)"%angular_z+" "+msg

        #self.ros_interface.command_velocity(linear_x, angular_z)
        return
    
def main(args):
    rospy.init_node('avoid_obstacle_02')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']
    min_lin_x = params['min_lin_x']
    max_lin_x = params['max_lin_x']
    rotate_fast_value = params['rotate_fast_value']
    linear_k = params['linear_k']
    k = params['k']
    min_linear_vel = params['min_linear_vel']
    rospy.loginfo("[robot_control]: ready")

    # Intialize the RobotControl object
    robotControl = RobotControl(pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body, min_lin_x, max_lin_x, rotate_fast_value, linear_k, k, min_linear_vel)

    r = rospy.Rate(6) # original 60Hz
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass


