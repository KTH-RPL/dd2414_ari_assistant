#!/usr/bin/env python3
import rospy

from actionlib import SimpleActionClient
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus
from dd2414_status_update import StatusUpdate
from pal_interaction_msgs.msg import TtsAction, TtsGoal



class TextSpeech:
    def __init__(self):
        self.language = "en_US"
        self.tts_client = SimpleActionClient("/tts", TtsAction)
        self.tts_client.wait_for_server()

    def action(self,goal):
        return self.tts_output(goal.goal)

    def preempted(self):
        pass

    def tts_output(self, sentence):
        """ Output the response through TTS. """
        goal = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text = sentence
        self.tts_client.send_goal_and_wait(goal)
        if self.tts_client.get_state() == GoalStatus.SUCCEEDED:
            return "Success"
        else:
            return "Failure"

if __name__=="__main__":
    rospy.init_node('text_speech')
    server = StatusUpdate(rospy.get_name(),TextSpeech)
    rospy.spin()