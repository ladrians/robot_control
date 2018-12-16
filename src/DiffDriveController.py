# -*- coding: utf-8 -*-
"""
@author: ladrians
"""
#!/usr/bin/python

import numpy as np
 
class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, min_speed, max_speed, min_omega, max_omega):
        # Check there parameters
        self.kp=0.5 # kp > 0
        self.ka=1.2 # ka > kp
        self.kb=0 # kb < 0
        self.MIN_SPEED = min_speed
        self.MAX_SPEED = max_speed
        self.MIN_OMEGA = min_omega
        self.MAX_OMEGA = max_omega
       
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or is close enough)
        """
        delta_x = (goal[0] - state[0]).item()
        delta_y = (goal[1] - state[1]).item()
        theta = state[2].item()
        pi = np.pi
        
        rho = np.sqrt(delta_x ** 2 + delta_y ** 2) # by definition
        beta = np.arctan2(delta_y, delta_x)
        self.alpha = beta - theta

        if self.alpha > (3 * pi / float(2)): # enforce constraints
            self.alpha = self.alpha - 2 * pi 
        elif self.alpha < -(3 * pi / float(2)):
            self.alpha = self.alpha + 2 * pi

        self.alpha = max(-pi/2 , min(pi/2 , self.alpha))
        
        beta = -theta - self.alpha # by definition

        v = max(min(self.kp * rho, self.MAX_SPEED),self.MIN_SPEED)
        omega_sign  = (self.ka * self.alpha + self.kb * beta) > 0
        omega = max(min(abs(self.ka * self.alpha + self.kb * beta), self.MAX_OMEGA),self.MIN_OMEGA)
        done = rho < 0.05 # 0.164 0.2

        return (v, omega if omega_sign else -omega, done) # check sign
