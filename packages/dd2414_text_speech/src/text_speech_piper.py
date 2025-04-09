#!/usr/bin/env python3
import rospy
import subprocess

from actionlib import SimpleActionClient
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus
from dd2414_status_update import StatusUpdate

class TextSpeechPiper:
    def __init__(self):
        self.language = "en_US"  # Default language
        model_path = os.path.expanduser('~/catkin_ws/src/piper/piper_models/es_ES-mls_9972-low.onnx')
        model_path = os.path.normpath(model_path)   
        self.tts = PiperTTS(model_path=model_path)

        self.tts_output("Estoy lista para ayudarte en lo que sea.")

    def tts_output(self, sentence):
        # Synthesize speech from the input sentence
        audio = self.tts.synthesize(sentence)
        
        # Play the audio buffer directly
        rospy.loginfo("Saying audio now")
        play_obj = sa.play_buffer(audio, num_channels=1, bytes_per_sample=2, sample_rate=22050)
        play_obj.wait_done()

if __name__ == "__main__":
    rospy.init_node('text_speech_piper')
    rospy.spin()
