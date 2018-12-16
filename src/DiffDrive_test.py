# -*- coding: utf-8 -*-
#!/usr/bin/env python

import numpy as np
import math
from DiffDriveController import DiffDriveController

min_omega = 0.0
max_omega = 1.6
min_vel = 0.0
max_vel = 0.8
dd = DiffDriveController(min_vel, max_vel, min_omega, max_omega)

pose = np.array([0,0,math.pi/4])
goal = np.array([0,0])
vel = dd.compute_vel(pose, goal)
print "0,0,math.pi/4:", vel

#new pose
pose = np.array([0,0,0])

goal = np.array([1,0])
vel = dd.compute_vel(pose, goal)
print "1,0:", vel

goal = np.array([1,1])
vel = dd.compute_vel(pose, goal)
print "1,1:", vel

goal = np.array([0,1])
vel = dd.compute_vel(pose, goal)
print "0,1:", vel

goal = np.array([-1,0])
vel = dd.compute_vel(pose, goal)
print "-1,0", vel

goal = np.array([-1,-1])
vel = dd.compute_vel(pose, goal)
print "-1,-1:", vel

goal = np.array([1,-1])
vel = dd.compute_vel(pose, goal)
print "1,-1:", vel


goal = np.array([-0.2,0.])
vel = dd.compute_vel(pose, goal)
print "-0.2,0.:", vel
