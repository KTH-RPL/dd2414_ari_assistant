#!/usr/bin/env python3
import rospy
from dd2414_status_update import StatusUpdate
import simpleaudio as sa
from actionlib import SimpleActionServer
import dd2414_text_speech.msg as tts



class TextMultilanguageSpeech:
    def __init__(self):
        self._as = SimpleActionServer('tts_multilanguage', tts.TextToSpeechMultilanguageAction, execute_cb=self.action_cb)

        self._as.start()  # Initialize ros service
        rospy.loginfo("Text-to-Speech Multilanguage Action Server is running.")

    def action_cb(self, goal):
        try:
            wave_obj = sa.WaveObject.from_wave_file("/tmp/tts_audio.mp3")
            play_obj = wave_obj.play()
            play_obj.wait_done()
        except Exception as e:
            rospy.logerr(f"Failed to play audio: {e}")

    def preempted(self):
        pass


if __name__=="__main__":
    rospy.init_node('text_multilanguage_speech')
    server = TextMultilanguageSpeech()
    rospy.spin()