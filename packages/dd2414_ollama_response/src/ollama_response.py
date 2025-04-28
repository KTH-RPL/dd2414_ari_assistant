#!/usr/bin/env python3
import os
import json
import rospy
import subprocess
import dd2414_brain_v2.msg as brain
import dd2414_text_speech.msg as tts 

from gtts import gTTS
from pydub import AudioSegment
from ollama import Client
from actionlib import SimpleActionClient
from dd2414_status_update import StatusUpdate

class OllamaResponse:
    def __init__(self):
        self.ip_ollama    = Client(host="http://130.229.134.112:11434")
        self.model_ollama = "mistral:latest"
        self.languages    = ["en:English","es:Spanish","de:German","fr:French","sv:Swedish"]
        self.system_promt = "You are an office assistant robot called ARI. Be concise and helpful. " \
        "These are the languages with their corresponding abreviation: " + ' '.join(self.languages) + ""
        self.result       = brain.BrainResult()
        self.tts_goal     = tts.TextToSpeechMultilanguageGoal()

        self.ac_ttsm = SimpleActionClient('text_multilanguage_speech', tts.TextToSpeechMultilanguageAction)
        self.ac_ttsm.wait_for_server()

        self.mp3_path = os.path.expanduser('/tmp/tts_audio.mp3')
        self.wav_path = os.path.expanduser('/tmp/tts_audio_wav.wav')
        
        rospy.loginfo("[Ollama Response]:Initialized")
        self.string_header = "[Ollama Response]:"

    def action(self,goal):
        language = json.loads(goal.in_dic)
        language = language["language"]
        
        return self.generate_response(goal.goal,language)

    def preempted(self):
        pass

    def tts_multilanguage_output(self,text,language):
        tts_audio = gTTS(text,lang=language)

        # Save the audio file
        tts_audio.save(self.mp3_path)

        # Convert to WAV
        audio = AudioSegment.from_mp3(self.mp3_path)
        audio.export(self.wav_path, format="wav")

        # Copy audio file to ARI
        command = ["sshpass", "-p", "pal", "scp", self.wav_path, "pal@192.168.128.28:/tmp/"]
        subprocess.run(command)

    def generate_response(self,user_input,language):
        completion = self.api.chat(
            model=self.model_ollama, 
            messages=[
            {"role":"system","content": self.system_promt + f" Respond in:{language}"},
            {"role":"user","content": user_input},
            ]
        )

        response = completion.message.content

        if language != "en":
            self.tts_multilanguage_output(response,language)
            self.tts_goal.data = self.wav_path
        else:
            self.tts_goal.data = response

        # Call the action server
        self.tts_goal.lang = language

        # Send audio goal
        self.ac_ttsm.send_goal(self.tts_goal)
        rospy.loginfo("[Ollama Response]:Response sent to TTS")

        self.result.result = "Success"

        return self.result
    
if __name__=="__main__":
    rospy.init_node('ollama_response',log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(),OllamaResponse)
    rospy.spin()




