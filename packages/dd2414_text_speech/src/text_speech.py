#!/usr/bin/env python3
import rospy

from actionlib import SimpleActionClient
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus
from dd2414_status_update import StatusUpdate
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import dd2414_brain_v2.msg as brain 



class TextSpeech:
    def __init__(self):
        self.language = "en_US"
        self.tts_client = SimpleActionClient("/tts", TtsAction)
        self.tts_client.wait_for_server()
        rospy.loginfo("[TTS            ]:Initialized")
        self.string_header = "[TTS            ]:"

    def action(self,goal):
        return self.tts_output(goal.goal)

    def preempted(self):
        pass

    def tts_output(self, sentence):
        
        """ Output the response through TTS. """
        result = brain.BrainResult()
        goal = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text = sentence
        self.tts_client.send_goal_and_wait(goal)
        if self.tts_client.get_state() == GoalStatus.SUCCEEDED:
            result.result = "Success"
        else:
            result.result = "Failure"
        return result

if __name__=="__main__":
    rospy.init_node('text_speech',log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(),TextSpeech)
    rospy.spin()