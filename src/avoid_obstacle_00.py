#!/usr/bin/env python
import rospy
import yaml
import numpy as np
import math
import sys

from RosInterface import ROSInterface
from DiffDriveController import DiffDriveController

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
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

    def process_measurements(self):
        """ 
        Basic code to run autonomously
        """
	# Check samples from https://github.com/markwsilliman/turtlebot/blob/master/goforward_and_avoid_obstacle.py

        sonar_meas = self.ros_interface.get_range()
        imu_meas = self.ros_interface.get_imu()

        if(sonar_meas == None):
            return
        elif(sonar_meas <= 30):
            pose = np.array([0,0,math.pi/4]) # emulate
            goal = np.array([0,0])
        else:
            pose = np.array([0,0,0])
            goal = np.array([1,0])

        vel = self.diff_drive_controller.compute_vel(pose, goal)
        self.vel = vel[0:2];
        self.ros_interface.command_velocity(vel[0], vel[1])
        rospy.loginfo_throttle(5, "robot_control velocity:"+ str(self.vel)+" sonar:"+str(sonar_meas))#+" imu:"+str(imu_meas))
        #rospy.loginfo("robot_control velocity:"+ str(self.vel)+" sonar:"+str(sonar_meas))#+" imu:"+str(imu_meas))
        return
    
def main(args):
    rospy.init_node('avoid_obstacle_01')

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

    r = rospy.Rate(15) # original 60Hz
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass


