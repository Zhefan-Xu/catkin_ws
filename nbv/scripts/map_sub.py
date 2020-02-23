#!/usr/bin/env python
import rospy
import numpy as np
#from tempfile import TemporaryFile


from nav_msgs.msg import OccupancyGrid



def callback(msg):
    #outfile = TemporaryFile()
    occ_map = msg.data
    map_width = msg.info.width
    map_height = msg.info.height
    occ_map = np.array(occ_map).reshape(map_height, map_width)
    occ_map = occ_map.T
    np.savetxt("map.csv", occ_map, delimiter=",")
    np.save('map', occ_map)

    #rospy.loginfo('Got Map!')
    #print('got map')



def map_listener():
    rospy.init_node('map_subsciber', anonymous = True)
    rospy.Subscriber('/map', OccupancyGrid, callback)

    rospy.spin()
if __name__ == '__main__':
    map_listener()