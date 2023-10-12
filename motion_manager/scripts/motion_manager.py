#!/usr/bin/env python

import time
import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

import math

pub = rospy.Publisher('/move_base_simple/goal', PoseStamped,queue_size=10)

def convert_to_posestamped(Point):

	x = Point.x
	y = Point.y
	z = 0.0
	x_o = 0.0
	y_o = 0.0
	z_o = math.sin(Point.z/2)
	w = math.cos(Point.z/2)
		
	
	return (x,y,z,x_o,y_o,z_o,w)

def callback(data):
	

	goal_pose = PoseStamped()
	goal_pose.header.frame_id = 'map'
	#goal_pose.header.navigator.get_clock().now().to_msg()
	


	x,y,z,x_o,y_o,z_o,w = convert_to_posestamped(data)

	goal_pose.pose.position.x = x
	goal_pose.pose.position.y = y
	goal_pose.pose.position.z = z
	goal_pose.pose.orientation.x = x_o
	goal_pose.pose.orientation.y = y_o
	goal_pose.pose.orientation.z = z_o
	goal_pose.pose.orientation.w = w
	


	pub.publish(goal_pose)
	

def motion_management_server():
	rospy.init_node('motion_managment')
	
	rospy.Subscriber('xyz',Point,callback)

	rospy.spin()

if __name__ == "__main__":
	#
	try:	
		motion_management_server()
	except rospy.ROSInterruptException:
		pass
