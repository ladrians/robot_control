#!/usr/bin/env python
import rospy
import yaml
import numpy as np
import math
import sys
import utility as util

from RosInterface import ROSInterface

from geometry_msgs.msg import Twist
from sensor_msgs.msg   import LaserScan

"""
Using a LIDAR to avoid obstacles, detail from  the sensor_msgs/LaserScan message:

# Laser scans angles are measured counter clockwise, with 0 facing forward
# (along the x-axis) of the device frame

Header header
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]
float32 time_increment   # time between measurements [seconds]
float32 scan_time        # time between scans [seconds]
float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]
float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units]
"""

# Constants
priority   = 100
node       = 'behavior_avoid'

avoid_check_rate = 0.5 # hz to wait until doing another move in case of go back

default_speed = 0.2
default_turn  = 0

clearance_safety_factor = 1.5 # defines how close the vehicle cuold come to an object
look_ahead_factor       = 3.5   # body width factor to look ahead 

min_rotation   = math.pi/180  # 1 degree
max_rotation   = min_rotation * 20
max_distance   = 99999


class Behaviour(object):
    # Topic's constants
    pub_topic      = 'action'   # Topic where every Behaviour publish its commands
    cmd_vel_topic  = 'cmd_vel'  # Topic where Arbiter publis the selected command
    scan2_topic    = 'scan'     # Topic from LaserScan sensor
    arbiter_time   = 0.21       # Duration at which arbiter select actions
    # Vehicle's contants
    body_width = 0.2524
    min_speed  = 0.01       # Minimum vehicle speed (under this value is same as 0)
    min_turn   = 0.01       # Idem for Turn

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

    def publish_action(self, speed, turn):
        self.ros_interface.command_velocity(speed, turn)

    def apply_avoidance_rules(self, distance_from_center, angle):
        p_angle = util.proportional_rotation(angle)
        action = "None"
        if self.inside_too_close_box(distance_from_center):
            # dangeorous distance, too close to turn, just try to go back
            action = "Back"
            rospy.loginfo("{}:{:.2f} {:.2f} dist:{:.2f}".format(action, angle, p_angle, distance_from_center))
            self.react_to_inside_too_close_box(p_angle)

        elif util.is_forward_left(angle):
            # obstacle to left, go right
            action = "go forward right"
            self.publish_action(default_speed, p_angle)
    
        elif util.is_forward_right(angle):
            # obstacle to right, go left
            action = "go forward left"
            self.publish_action(default_speed, p_angle)

        elif util.is_backwards_left(angle):
            action = "obstacle backwards left"

        elif util.is_backwards_right(angle):
            action = "obstacle backwards right"

        else:
            action = "unknown"

        #rospy.loginfo("{}:{:.2f} {:.2f} dist:{:.2f}".format(action, angle, p_angle, distance_from_center))

    
    def react_to_inside_too_close_box(self,angle):
        # go back and turn 180 degrees
        self.publish_action(-default_speed*2, 0)
        rate = rospy.Rate(avoid_check_rate) 
        rate.sleep()
        if util.is_forward_right(angle):
            sign = -1        
        else:
            sign = 1
        z = math.pi*1.6*sign
        rospy.loginfo('Back angle:'+str(angle)+' new angle:'+str(z))
        self.publish_action(0, z) # around 5 rad/s
        rate.sleep()
 
    def inside_too_close_box(self, distance_from_center):
        forward_box_length, forward_box_width = self.forward_box()
        return distance_from_center < forward_box_width
        
    def get_closest_obstacle(self, scan):
        # returns closest point to obstacle inside a vehicle's 'forward box' 
        # forward box is a rectangle ahead of the vehicle that if there is an obstacle
        # inside it it's necessary to do an action in order to avoid it
        forward_box_length, forward_box_width = self.forward_box()

        is_inside    = False
        min_distance = max_distance
        min_i = 0
        min_angle = 0

        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if util.is_within_range(angle) and util.is_valid_distance(distance):
                #print i, distance, to_degrees(angle), angle
                if distance < min_distance:
                    if point_is_inside_box(i, distance, scan, forward_box_length, forward_box_width, util.to_degrees(angle)):
                        is_inside = True
                        min_distance = distance
                        min_i = i
                        min_angle = angle
    
        #angle = angle_from_scan(min_i, scan)
        #angle_degrees = angle*180/math.pi
        #print min_i, angle_degrees, min_distance

        return is_inside, min_distance, min_angle

    def forward_box(self):
        # returns the Vehicle 'forward box' 
        # to do: in future versions forward box should depend on vehicle's speed
        #        more speed -> larger forward box
        length = Behaviour.body_width*look_ahead_factor
        width  = clearance_safety_factor*Behaviour.body_width/2
        return length, width

    def check_action():
        pass

    def process_measurements(self):
        """ 
        Basic code to run autonomously
        """

        scan = self.ros_interface.get_scan()
        if(scan == None):
            return

        action = "None"
        angular_z = 0
        linear_x = 0.3

        is_inside, distance_from_center, angle = self.get_closest_obstacle(scan)

        # only do something if obstacle is inside forward box
        if is_inside:
            #print 'point inside ', distance_from_center, angle
            self.apply_avoidance_rules(distance_from_center, angle)
        else:
            self.publish_action(linear_x, angular_z)	

        return
    
def point_is_inside_box(i, distance, scan, forward_box_length, forward_box_width, degrees):
    point_angle = angle_from_scan(i, scan)
    front, side = distance_from_vehicle(distance, point_angle)
    if front > 0 and front <= forward_box_length and side <= forward_box_width:
        t = "OK"
        ret = True
        #print "i:{:3.0f} z:{:1.2f}({:3.0f}) {} dist:{:.2f} front({:.2f}<{:.2f}) side:({:.2f}<{:.2f})".format(i, point_angle, degrees, t, distance, front, forward_box_length, side, forward_box_width)
    else:
        t = "NO"
        ret = False
    return ret

def angle_from_scan(i, scan):
    return scan.angle_min + i*scan.angle_increment
    #return math.pi/2 + scan.angle_min + i*scan.angle_increment

def distance_from_vehicle(distance_from_center, angle):
    distance_to_vehicle_front = abs(distance_from_center*math.cos(angle)) # Warning, LIDAR rotated and taking 180 degrees forward, sign does not matter
    distance_to_vehicle_side  = abs(distance_from_center*math.sin(angle))
    return distance_to_vehicle_front, distance_to_vehicle_side

def main(args):
    rospy.init_node('avoid_obstacle_04')

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
    rospy.loginfo("[avoid_obstacle_04]: ready")

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body)

    r = rospy.Rate(20) # original 60Hz
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    robotControl.publish_action(0, 0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass

