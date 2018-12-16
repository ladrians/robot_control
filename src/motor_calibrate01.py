#!/usr/bin/env python
"""
Sample code to calibrate DC Motors, sample taken from
https://github.com/Bartman0/robotics-capstone/blob/master/src/robot_control/src/RobotControl.py
"""
import rospy

import yaml
import numpy as np

import sys

from RosInterface import ROSInterface

class RobotControl(object):
    """
    Robot interface
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        """

        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

        self.time_init = -1.0
        

    def process_measurements(self):
        """ 
        This function is called at 60Hz
        """

        # set init time, and start motor
        #if 0 == 1:
        if self.time_init == -1.0:
            print "set init time"
            self.time_init = rospy.get_time()
        self.time_measurement = rospy.get_time()
        # stop motor when 1 second has passed
        diff_time = (self.time_measurement - self.time_init)
        #print("init: {}, measurement: {}, diff: {}".format(self.time_init, self.time_measurement, diff_time))
        if diff_time >= 1:
            print "stop speed"
            self.ros_interface.command_velocity(0, 0)
        else:
            print "set speed"
            self.ros_interface.command_velocity(0.3, 0)
        
        return
    
def main(args):
    rospy.init_node('motor_calibrate01')

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

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body)

    # Call process_measurements at 60Hz
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
