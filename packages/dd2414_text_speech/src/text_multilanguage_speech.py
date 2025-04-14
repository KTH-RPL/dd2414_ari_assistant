#!/usr/bin/env python3
import rospy
from dd2414_status_update import StatusUpdate
import simpleaudio as sa
from actionlib import SimpleActionServer
import dd2414_text_speech.msg as tts
from pydub import AudioSegment
# from io import BytesIO
from gtts import gTTS
import os



class TextMultilanguageSpeech:
    def __init__(self):
        self._as = SimpleActionServer('text_multilanguage_speech', tts.TextToSpeechMultilanguageAction, execute_cb=self.action_cb)

        self._as.start()  # Initialize ros service

        self.mp3_path = os.path.expanduser('/tmp/tts_audio.mp3')
        self.wav_path = os.path.expanduser('/tmp/tts_audio.mp3')
        rospy.loginfo("Text-to-Speech Multilanguage Action Server is running.")

    def action_cb(self, goal):
        rospy.loginfo("Received audio to play from LLM.")
        try:
            # Get the text and language from the goal
            text = goal.data
            lang = goal.lang

            # Perform TTS using gTTS (Google Text-to-Speech)
            tts_audio = gTTS(text, lang)

            # Save the audio file
            tts_audio.save(self.mp3_path)

            # Convert to WAV
            audio = AudioSegment.from_mp3(self.mp3_path)
            audio.export(self.wav_file_path, format="wav")

            # Now load into simpleaudio and play
            wave_obj = sa.WaveObject.from_wave_file(self.wav_file_path)
            play_obj = wave_obj.play()
            play_obj.wait_done()

            rospy.loginfo("TTS playback completed.")
            self._as.set_succeeded()

        except Exception as e:
            rospy.logerr(f"Failed to play audio: {e}")
            self._as.set_aborted()


if __name__=="__main__":
    rospy.init_node('text_multilanguage_speech')
    server = TextMultilanguageSpeech()
    rospy.spin()