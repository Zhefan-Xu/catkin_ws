#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import time

def make_move():
	rospy.init_node('make_movement', anonymous = True)
	pub = rospy.Publisher('/cmd_vel', Twist)
	rate = rospy.Rate(10)
	t0 = time.clock()
	msg = Twist()
	#rot_q = msg.pose.pose.position.x
	#(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z])
	while not rospy.is_shutdown():
		
		msg.linear.x = 0.1
		print(time.clock() - t0)
		if (time.clock() - t0 > 5):
			msg.linear.x = 0
		pub.publish(msg)
		#rate.sleep

if __name__ == '__main__':
	try:
		make_move()
	except rospy.ROSInterruptException:
		pass