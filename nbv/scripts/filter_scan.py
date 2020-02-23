#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from math import *
import numpy as np
import copy

ranges_filter = []
intensities_filter = []

#copy the range and intensities of "/scan" topic to "ranges_filter" and "intensities_filter" 
#you need to convert them to "list" as "data.ranges" and "data.intensities" are "tuple"
def callback_scan(data):
    global ranges_filter, intensities_filter

    len(data.ranges) #360
    len(data.intensities) #360

    ranges_filter = copy.copy(data.ranges)
    intensities_filter = copy.copy(data.intensities)

    #convert them to list
    ranges_filter = list(ranges_filter)
    intensities_filter = list(intensities_filter)

     #filtering those angles that I do not want them (based on the question)
    for x in range(45, 180):
        ranges_filter[x] = 0
        intensities_filter[x] = 0

    for y in range(180, 315):
        ranges_filter[y] = 0
        intensities_filter[y] = 0

#define a new topic called "filterScan" to store all laser scanner data
rospy.init_node('laser_scan_filter')

scan_pub = rospy.Publisher('modified_scan', LaserScan, queue_size=50)

rospy.Subscriber("/scan", LaserScan, callback_scan) 

#it is based on the type of laser scanner (length of data.ranges)
num_readings = 360
laser_frequency = 60

count = 0
r = rospy.Rate(1.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    filterScan = LaserScan()

    filterScan.header.stamp = current_time
    filterScan.header.frame_id = 'base_scan'
    filterScan.angle_min = 0 # start angle of the scan [rad]
    filterScan.angle_max = math.pi * 2  # end angle of the scan [rad]
    filterScan.angle_increment = 0.0174532923847 # angular distance between measurements [rad]
    filterScan.time_increment = 2.98699997074e-05 # time between measurements [seconds]
    filterScan.range_min = 0.0 # minimum range value [m]
    filterScan.range_max = 3.5 # maximum range value [m]

    filterScan.ranges = []
    filterScan.intensities = []


    for i in range(0, num_readings-1):
        filterScan.ranges = copy.copy(ranges_filter)

        filterScan.intensities = copy.copy(intensities_filter)

    scan_pub.publish(filterScan)
    r.sleep()