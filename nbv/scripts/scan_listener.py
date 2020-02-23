#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


def callback(msg):
	global LaserScan_msg
	global ranges
	global intensities
	LaserScan_msg = msg
	ranges = msg.ranges
	intensities = msg.intensities

	#print('Got')


def scan_modifier():
	rospy.init_node('scan_modifier')
	rospy.Subscriber('scan', LaserScan, callback)
	pub = rospy.Publisher('modified_scan', LaserScan, queue_size = 10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		ranges_modified = np.zeros(360, dtype = float)
		intensities_modified = np.zeros(360, dtype = float)
		try:
			#ranges_modified[0:359] = ranges[0:359]
			ranges_modified[315:360] = ranges[315:360]
			ranges_modified[0: 45] = ranges[0: 45]
			intensities_modified[0:359] = intensities[0:359]
			#intensities_modified[271:360] = intensities[271:360]

			#ranges_modified = np.array(ranges)
			##index = np.argwhere(ranges_modified > 1)
			#ranges_modified[index] = 1
			#intensities_modified = np.array(intensities)
			#intensities_modified[index] = 0
		except (NameError):
			continue

		msg = LaserScan_msg
		msg.ranges = ranges_modified
		msg.intensities = intensities_modified
		pub.publish(msg)
		rate.sleep()

	rospy.spin()




if __name__ == '__main__':
    scan_modifier()