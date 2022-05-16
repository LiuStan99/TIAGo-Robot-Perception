#!/usr/bin/env python

#import ROS headers
from mimetypes import init
import rospy
import actionlib

#import PAL Robotics custom headers
from pal_interaction_msgs.msg import TtsAction, TtsGoal

def event(event):
	return {
	1 : 'TTS_EVENT_INITIALIZATION',
	2 : 'TTS_EVENT_SHUTDOWN',
	4 : 'TTS_EVENT_SYNCHRONIZATION',
	8 : 'TTS_EVENT_FINISHED_PLAYING_UTTERANCE',
	16 : 'TTS_EVENT_MARK',
	32 : 'TTS_EVENT_STARTED_PLAYING_WORD',
	64 : 'TTS_EVENT_FINISHED_PLAYING_PHRASE',
	128 : 'TTS_EVENT_FINISHED_PLAYING_SENTENCE'
	}[event]

def feedbackCb(feedback):
	print("event type: " + event(feedback.event_type))
	print("timestamp: " + str(feedback.timestamp))
	print("current word: " + feedback.text_said)
	print("next word: " + feedback.next_word)
	print("-")
'''
class server:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('name_placeholder_server', DoSomethingAction, self.execute, auto_start=False)
		self.server.start()
	def execute(self, goal):

		self.server.set_succeeded
		

'''
def config_speech(situation):
	"""
	goal has integer (situation)
	0 : welcome,
	1 : introduction,
	2 : point of interest 1,
	3 : point of interest 2,
	4 : point of interest 3,
	5 : ending,
	6 : not working properly
	"""
	situations = {0 : "/welcome.txt",
				1 : "/introduction.txt",
				2 : "/point_of_interest1.txt",
				3 : "/point_of_interest2.txt",
				4 : "/point_of_interest3.txt",
				5 : "/ending.txt",
				6 : "/not_working_properly.txt"}
	
	file = data_folder + situations[situation]
	with open(file, 'r') as file:
		text = file.read().replace('\n', ' ')
	return text		

	
def initiate_speech(text):
	print("---")
	goal.rawtext.text = text
	goal.rawtext.lang_id = "EN"
	client.send_goal(goal, feedback_cb=feedbackCb)
	client.wait_for_result()
	res = client.get_result()
	print("text: " + res.text)
	print("warning/error msgs: " + res.msg)
	print("---")

if __name__ == '__main__':
	rospy.init_node('hri_tts_client')
	client = actionlib.SimpleActionClient('tts_to_soundplay', TtsAction)
	rospy.loginfo("Waiting for Server")
	client.wait_for_server()
	rospy.loginfo("Reached Server")
	goal = TtsGoal()

	data_folder = "/home/stijn/tiago_dual_public_ws/src/cor_mdp_tiago/hri/speeches"

	run = True
	done = False
	while run:
		if rospy.Time.now() > rospy.Time(10) and done == False:
			msg_received = True
		else:
			msg_received = False


		if msg_received:
			msg_goal = 1 # TODO goal of action that is received
			initiate_speech(config_speech(msg_goal))
			done = True
		
		if rospy.Time.now() > rospy.Time(100):
			print("Time is above 100 sec simulated time, time to sleep")
			run = False