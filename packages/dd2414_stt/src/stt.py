#!/usr/bin/env python3
import wave
import json
import rospy
import whisper

from std_msgs.msg import String
from deep_translator import GoogleTranslator
from audio_common_msgs.msg import AudioData

class STT:
    def __init__(self):
        self.rate          = rospy.Rate(1)
        self.string_header = "[STT            ]:"
        self.languages     = ["en","es","de","fr","sv"]
        self.stt_model     = whisper.load_model("turbo")
        self.verbose       = rospy.get_param("/verbose",False)
        self.transcript    = {}
        self.msg_data      = ""

        # Audio parameters – adjust according to your setup
        #self.stt_listen   = False
        self.stt_filename = f"/tmp/ARI_stt.wav"
        self.SAMPLE_RATE  = 16000  # Hz
        self.SAMPLE_WIDTH = 2      # 2 bytes = 16-bit audio
        self.CHANNELS     = 1      # Mono
        self.stt_result   = ""
        self.stt_language = "en"

        # Publishers
        self.stt_pub = rospy.Publisher("/stt/transcript",String,queue_size=10)

        # Subscribers
        rospy.Subscriber("/audio/speech", AudioData, self.stt_audio, queue_size=1)
        
        rospy.sleep(3) 
        rospy.loginfo("[STT            ]:Initialized")

    def stt_audio(self,msg):
        if not msg.data:
            self.msg_data = None
            return
        
        else:
            self.msg_data = msg.data
            self.process_audio(self.msg_data)

    def process_audio(self,msg_data):

        with wave.open(self.stt_filename, 'w') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.SAMPLE_WIDTH)
            wf.setframerate(self.SAMPLE_RATE)
            wf.writeframes(msg_data)  # msg.data is a bytes object
            wf.close
        #rospy.loginfo(f"Saved audio to {self.stt_filename}")
        result       = self.stt_model.transcribe(self.stt_filename)
        stt_result   = result["text"]
        stt_language = result["language"]

        if stt_language != "en":
            result_translated = GoogleTranslator(source=stt_language, target='en').translate(stt_result)
        else:
            result_translated = stt_result

        self.transcript["text"]        = stt_result
        self.transcript["translation"] = result_translated
        self.transcript["language"]    = stt_language

        self.stt_pub.publish(json.dumps(self.transcript))
        rospy.loginfo(f"[STT            ]:Text:{stt_result}, Translation:{result_translated}, Language:{stt_language}")

if __name__ == '__main__':
    try:
        rospy.init_node("sst",log_level=rospy.INFO)
        stt_node = STT()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Node interrupted.")
        




