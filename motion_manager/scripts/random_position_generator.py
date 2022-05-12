#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Point

import random 
import math

def generator():

	#pub = rospy.Publisher('chatter',String,queue_size=10)
	pub = rospy.Publisher('xyz',Point,queue_size=10)
	rospy.init_node('talker',anonymous=True)
	
	rate  = rospy.Rate(0.1)

	while not rospy.is_shutdown():
		#hello_str = "hello world %s" % rospy.get_time()
		#rospy.loginfo(hello_str)
		#pub.publish(hello_str)
		x = random.uniform(-2.0,2.0)
		y = random.uniform(-2.0,2.0)
		theta = random.uniform(0,2*math.pi)

		position = Point()

		position.x = x
		position.y = y
		position.z = theta			
		
		rospy.loginfo(position)
		pub.publish(position)
		rate.sleep()

if __name__ == "__main__":
	try:
		generator()
	except rospy.ROSInterruptException:
		pass
