#!/usr/bin/env python3

import sys
import rospy
from std_srvs.srv import Empty
from actionlib import SimpleActionClient
from hri_msgs.msg import LiveSpeech
from pal_navigation_msgs.srv import Acknowledgment, AcknowledgmentRequest
from pal_interaction_msgs.msg import TtsAction, TtsGoal

class Mapper(object):
    
    def __init__(self):
        rospy.loginfo("Talker::init node")

        # Connect to the text-to-speech action server
        self.tts = SimpleActionClient('/tts', TtsAction)
        self.tts.wait_for_server()
        rospy.loginfo("RobotBase::Connected to TTS action client")

        # Subsrcribe to speech topic
        self.speech_sub = rospy.Subscriber('/humans/voices/anonymous_speaker/speech', LiveSpeech, self.speech_cb)
        rospy.loginfo("RobotBase::Subscribed to " + self.speech_sub.resolved_name)

        # Connect to mapping services
        rospy.wait_for_service('pal_navigation_sm')
        rospy.wait_for_service('/pal_map_manager/start_map')
        rospy.wait_for_service('/pal_map_manager/stop_map')

        self.navigation_mode = rospy.ServiceProxy('pal_navigation_sm', Acknowledgment)
        self.start_map = rospy.ServiceProxy('pal_map_manager/start_map', Empty)
        self.stop_map = rospy.ServiceProxy('pal_map_manager/stop_map', Empty)
        self.change_map = rospy.ServiceProxy('pal_map_manager/change_map', Acknowledgment)
        

    def start_mapping(self):
        # Announce start of mapping
        self.talk("Starting mapping")
        print("Sending request to change navigation mode to mapping")

        # Set navigation mode to mapping
        request = AcknowledgmentRequest()
        request.input = 'MAP'
        response = self.navigation_mode(request)


    def end_mapping(self):
        # Announce end of mapping
        self.talk("Stopping mapping")
        print("Sending request to change navigation mode to localisation")

        # Set navigation mode back to localisation
        request = AcknowledgmentRequest()
        request.input = 'LOC'
        response = self.navigation_mode(request)


    def talk(self, text):
        rospy.loginfo("I'll say: " + text)
        
        # Create a goal to say our sentence
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"

        # Send the goal and wait
        self.tts.send_goal_and_wait(goal)

        
    def speech_cb(self, data):
        # When speech is published, either start or end mapping
        if data.final == "stop exploring":
            self.end_mapping()

        if data.final == "start exploring":
            self.start_mapping()


if __name__ == '__main__':

    try:
        # Initialize the node
        rospy.init_node('mapper_node', anonymous=True)
        mapper = Mapper()

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
