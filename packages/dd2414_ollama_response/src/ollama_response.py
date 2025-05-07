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
from std_msgs.msg import String
from dd2414_status_update import StatusUpdate

class OllamaResponse:
    def __init__(self):
        self.ip_ollama    = Client(host="http://localhost:11434")#Client(host="http://130.229.183.186:11434")
        self.model_ollama = "mistral:latest"
        self.languages    = {"en":"English","es":"Spanish","de":"German","fr":"French","sv":"Swedish"}
        self.system_promt = "You are an office assistant robot called ARI. Be concise and helpful. "
        self.result       = brain.BrainResult()
        self.tts_goal     = tts.TextToSpeechMultilanguageGoal()

        self.brain_person_name = "unknown"

        rospy.Subscriber("/brain/user_name",String,self.update_brain_person)

        # Action client of TTS Multilanguages
        self.ac_ttsm = SimpleActionClient('text_multilanguage_speech', tts.TextToSpeechMultilanguageAction)
        self.ac_ttsm.wait_for_server()

        self.mp3_path = os.path.expanduser('/tmp/tts_audio.mp3')
        self.wav_path = os.path.expanduser('/tmp/tts_audio_wav.wav')
        
        rospy.loginfo("[Ollama Response]:Initialized")
        self.string_header = "[Ollama Response]:"

    def update_brain_person(self,msg):
        if not msg.data:
            self.brain_person_name = "unknown"
        else:
            self.brain_person_name = msg.data

    def action(self,goal):
        try:
            dictonary = json.loads(goal.in_dic)
            language  = dictonary["language"]
            intent    = dictonary["intent"]
            phrase    = dictonary["phrase"]
        
            return self.generate_response(phrase,intent,language)
        
        except Exception as e:
            rospy.logerr(f"Empty Goal: {e}")
            self.result.result = "Failure"
            
            return self.result

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

    def generate_response(self,phrase,intent,language):
        name_promt = ""
        if intent in ["greet","goodbye"] and self.brain_person_name != "unknown":
            name_promt = "My name is: " + self.brain_person_name + ". Include it in the response."

            completion = self.api.chat(
                model=self.model_ollama, 
                messages=[
                {"role":"system","content": self.system_promt + name_promt +f" Respond in:{str(self.languages[language])}"},
                {"role":"user","content": phrase},
                ]
            )

            response = completion.message.content
            rospy.loginfo(f"[Response Ollama]:Response : {response}")
        
        else:
            response = phrase

        if language != "en":
            self.tts_multilanguage_output(response,language)
            self.tts_goal.data = self.wav_path
        else:
            self.tts_goal.data = response

        # Call the action server
        self.tts_goal.lang = language

        # Send audio goal
        rospy.loginfo("[Ollama Response]:Response sent to TTS")
        self.ac_ttsm.send_goal_and_wait(self.tts_goal)

        self.result.result = "Success"

        return self.result
    
if __name__=="__main__":
    rospy.init_node('ollama_response',log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(),OllamaResponse)
    rospy.spin()




