#!/usr/bin/env python3
import os
import json
import rospy
import simpleaudio as sa
import dd2414_text_speech.msg as tts

from gtts import gTTS
from pydub import AudioSegment
from actionlib import SimpleActionServer, SimpleActionClient
from std_msgs.msg import String
from dd2414_status_update import StatusUpdate
from pal_interaction_msgs.msg import TtsAction, TtsGoal
# from io import BytesIO

class TextMultilanguageSpeech:
    def __init__(self):
        self._as = SimpleActionServer('text_multilanguage_speech', tts.TextToSpeechMultilanguageAction, execute_cb=self.action_cb)

        self._as.start()  # Initialize ros service

        self.tts_client = SimpleActionClient("/tts",TtsAction)
        self.tts_client.wait_for_server()
        self.rate = rospy.Rate(10)
        self.talking = ""

        # Publishers
        self.speek_pub = rospy.Publisher("/tts/ARI_speeking",String,queue_size=10)

        self.mp3_path = os.path.expanduser('/tmp/tts_audio.mp3')
        rospy.loginfo("Text-to-Speech Multilanguage Action Server is running.")

    def tts_output(self,text):
        self.talking ="speaking"
        self.speek_pub.publish(self.talking)
        goal                 = TtsGoal()
        goal.rawtext.lang_id = "en_US"
        goal.rawtext.text    = text
        self.tts_client.send_goal_and_wait(goal)

    def run(self):
        self.speek_pub.publish(self.talking)
        self.rate.sleep()

    def action_cb(self, goal):
        rospy.loginfo("Received audio to play from LLM.")

        try:
            # Get the text and language from the goal
            text_path = goal.data 
            lang      = goal.lang

            if lang == "en":
                # Use PAL TTS
                rospy.loginfo(text_path)
                self.tts_output(text_path)
            else:
                # Now load into simpleaudio and play
                self.talking = "speaking"
                self.speek_pub.publish(self.talking)
                audio = AudioSegment.from_wav(text_path)
                louder_audio = audio + 6
                louder_audio.export(text_path, format="wav")
                wave_obj = sa.WaveObject.from_wave_file(text_path)

                play_obj = wave_obj.play()
                play_obj.wait_done()
                rospy.sleep(1)

            rospy.loginfo("TTS playback completed.")
            rospy.sleep(1)
            self.talking = ""
            self._as.set_succeeded()

        except Exception as e:
            rospy.logerr(f"Failed to play audio: {e}")
            self._as.set_aborted()


if __name__=="__main__":
    rospy.init_node('text_multilanguage_speech')
    server = TextMultilanguageSpeech()
    while not rospy.is_shutdown():
        server.run()
        
    