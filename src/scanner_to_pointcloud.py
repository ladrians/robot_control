#!/usr/bin/env python  
"""
Based on http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData
Transform the LIDAR to a PointCloud
http://answers.ros.org/question/202787/using-pointcloud2-data-getting-xy-points-in-python/
"""
import roslib
from sensor_msgs.msg import LaserScan
from geometry.msgs import PointStamped
from sensor.msgs import PointCloud
from laser_geometry import LaserProjection
#import sensor_msgs.point_cloud2 as pc2
import math
import rospy
import tf

def scan_callback(scan):
    frame_id = scan.header.frame_id

    try:
        projector.transformLaserScanToPointCloud(
          "map", scan, cloud, listener)
    except Exception as e:
      print "exception",e
      return

    scan_pub_.publish(cloud);
    '''
    try:
      (trans,rot) = listener.lookupTransform("/scanner", "/body", rospy.Time(1.0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
      print "exception",e
      return
    print trans, rot
    '''


if __name__ == '__main__':
    rospy.init_node('scanner_to_pointcloud')

    rospy.Subscriber('scan', LaserScan, scan_callback)
    listener = tf.TransformListener()
    projector = LaserProjection()
    scan_pub = rospy.Publisher('pointcloud', PointCloud, queue_size=10)
    cloud = PointCloud()

rospy.spin()
