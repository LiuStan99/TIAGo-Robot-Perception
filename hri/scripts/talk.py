#import ROS headers
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

if __name__ == '__main__':
	rospy.init_node('hri_tts_client')
	client = actionlib.SimpleActionClient('tts_to_soundplay', TtsAction)
	rospy.loginfo("Waiting for Server")
	client.wait_for_server()
	rospy.loginfo("Reached Server")
	goal = TtsGoal()
	while not rospy.is_shutdown():
		if True:
			text = "hallo allemaal, welkom"
		print("---")
		goal.rawtext.text = text
		goal.rawtext.lang_id = "NL"
		client.send_goal(goal, feedback_cb=feedbackCb)
		client.wait_for_result()
		res = client.get_result()
		print("text: " + res.text)
		print("warning/error msgs: " + res.msg)
		print("---")