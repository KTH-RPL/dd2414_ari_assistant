#!/usr/bin/env python3
import rospy
import wave
import datetime
from audio_common_msgs.msg import AudioData

class SpeechText:
    def __init__(self):
        rospy.init_node('audio_saver')
        rospy.loginfo("[Speech Text]: Audio saver node started. Listening for audio...")
        # Audio parameters – adjust according to your setup
        self.SAMPLE_RATE = 16000  # Hz
        self.SAMPLE_WIDTH = 2     # 2 bytes = 16-bit audio
        self.CHANNELS = 1         # Mono

        rospy.Subscriber("/audio/speech", AudioData, self.audio_callback)


    #Callback to save audio
    def audio_callback(self,msg):
        # Generate a unique filename
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"/tmp/ARI_stt.wav"

        # Open WAV file for writing
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.SAMPLE_WIDTH)
            wf.setframerate(self.SAMPLE_RATE)
            wf.writeframes(msg.data)  # msg.data is a bytes object

        rospy.loginfo(f"[Speech Text]: Saved audio to {filename}")

if __name__ == '__main__':
    try:
        SpeechText()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Chatbos: ROS Node interrupted.")
