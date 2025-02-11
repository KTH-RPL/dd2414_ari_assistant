#!/usr/bin/env python3

import sys
import rospy
from actionlib import SimpleActionClient
from hri_msgs.msg import LiveSpeech
from pal_interaction_msgs.msg import TtsAction, TtsGoal

class Talker(object):
    
    def __init__(self):
        rospy.loginfo("Talker::init node")

        # Connect to the text-to-speech action server
        self.tts = SimpleActionClient('/tts', TtsAction)
        self.tts.wait_for_server()
        rospy.loginfo("RobotBase::Connected to TTS action client")

        self.speech_sub = rospy.Subscriber('/humans/voices/anonymous_speaker/speech', LiveSpeech, self.speech_cb)
        rospy.loginfo("RobotBase::Subscribed to " + self.speech_sub.resolved_name)

    def talk(self, text):


        rospy.loginfo("I'll say: " + text)
        
        # Create a goal to say our sentence
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"

        # Send the goal and wait
        self.tts.send_goal_and_wait(goal)

        rate = rospy.Rate(5)
        rate.sleep()
        
    def speech_cb(self, data):

        # When speech is published, say the text of the speech
        if data.final:
            self.talk(data.final)


if __name__ == '__main__':

    try:
        # Initialize the node
        rospy.init_node('talking', anonymous=True)
        talker = Talker()

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

