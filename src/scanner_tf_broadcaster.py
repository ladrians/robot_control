#!/usr/bin/env python  
"""
Based on http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29
Transform the LIDAR position
"""
import roslib
#roslib.load_manifest('scanner_tf')
import math
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('scanner_tf')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, math.pi, 1.0), # inverted scanner
                         rospy.Time.now(),
                         "scanner",
                         "body")
        rate.sleep()
