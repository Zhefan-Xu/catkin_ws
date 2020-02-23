#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

def callback(msg):
	




def modified_scan_sub():
	rospy.init_node('modified_scan_sub')
	rospy.Subscriber('modified_scan', LaserScan, callback)
	rospy.spin()


if __name__ = '__main__':
	modified_scan_sub()