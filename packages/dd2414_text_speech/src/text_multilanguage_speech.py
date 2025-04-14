#!/usr/bin/env python3
import rospy
from dd2414_status_update import StatusUpdate
import simpleaudio as sa
from actionlib import SimpleActionServer
import dd2414_text_speech.msg as tts
from pydub import AudioSegment
from io import BytesIO




class TextMultilanguageSpeech:
    def __init__(self):
        self._as = SimpleActionServer('text_multilanguage_speech', tts.TextToSpeechMultilanguageAction, execute_cb=self.action_cb)

        self._as.start()  # Initialize ros service
        rospy.loginfo("Text-to-Speech Multilanguage Action Server is running.")

    def action_cb(self, goal):
        rospy.loginfo("Received audio to play from LLM.")
        try:
            mp3_bytes = bytes(goal.data)
            audio = AudioSegment.from_file(BytesIO(mp3_bytes), format="mp3")

            # Export to BytesIO as WAV
            wav_io = BytesIO()
            audio.export(wav_io, format="wav")
            wav_io.seek(0)

            # Now load into simpleaudio
            wave_obj = sa.WaveObject.from_wave_read(sa.WaveObject._wave_open(wav_io))
            play_obj = wave_obj.play()
            play_obj.wait_done()
            self._as.set_succeeded()
        except Exception as e:
            rospy.logerr(f"Failed to play audio: {e}")
            self._as.set_aborted()


if __name__=="__main__":
    rospy.init_node('text_multilanguage_speech')
    server = TextMultilanguageSpeech()
    rospy.spin()