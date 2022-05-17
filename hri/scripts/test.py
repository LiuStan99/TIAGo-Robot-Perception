#!/usr/bin/env python

#import ROS headers
import rospy
import actionlib
import roslib
roslib.load_manifest('hri')

from hri.msg import speechAction, speechGoal

if __name__ == '__main__':
    rospy.init_node('hri_test_speech_client')
    client = actionlib.SimpleActionClient('speech', speechAction)
    rospy.loginfo("Waiting for Speech Server")
    client.wait_for_server()
    rospy.loginfo("Reached Speech Server")
    goal = speechGoal()
    run = True
    done = False
    while run:
        if rospy.Time.now() > rospy.Time(15) and done == False:
            rospy.loginfo("sending action ...")
            goal.situation = 1
            client.send_goal(goal)
            client.wait_for_server()
            done = True
        
        if rospy.Time.now() > rospy.Time(100):
            print("Time is above 100 sec simulated time, time to sleep")
            run = False
