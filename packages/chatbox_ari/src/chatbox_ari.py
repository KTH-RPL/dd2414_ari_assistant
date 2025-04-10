#!/usr/bin/env python3
import re
import os
import json
import wave
import rospy
import whisper
import paramiko

from gtts import gTTS
from ollama import Client
from actionlib import SimpleActionClient
from std_msgs.msg import String
from hri_msgs.msg import LiveSpeech
from audio_common_msgs.msg import AudioData
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from dd2414_text_speech.msg import TextToSpeechMultilanguageAction




class ChatboxARI:
    def __init__(self):
        rospy.init_node("chatbox_ari",log_level=rospy.INFO)
        self.rate              = rospy.Rate(1)
        self.listen            = True
        self.string_header     = "[LLM            ]:"
        self.stop              = False
        self.brain_state_data  = "Busy"
        self.brain_person_name = "unknown"
        self.brain_msg_time    = rospy.Time.now().to_sec()
        self.language          = "en_US"
        self.model_ollama      = "mistral:latest"
        self.stt_model         = whisper.load_model("base")
        self.verbose           = rospy.get_param("/verbose",False)

        # Audio parameters – adjust according to your setup
        self.stt_listen   = False
        self.stt_filename = f"/tmp/ARI_stt.wav"
        self.SAMPLE_RATE  = 16000  # Hz
        self.SAMPLE_WIDTH = 2      # 2 bytes = 16-bit audio
        self.CHANNELS     = 1      # Mono
        self.stt_result   = ""
        self.stt_language = "en"

        self.intents = {         #Action/Split
            "greet"              :[False,False],
            "remember user"      :[False,True ],
            "goodbye"            :[False,False],
            "follow user"        :[True ,False],
            "provide information":[False,False],
            "find object"        :[True ,True ],
            "go to"              :[True ,True ],
            "explore"            :[True ,False],
            "translate"          :[True ,True ],
            "other"              : False      }
        
        self.intents_description  = [
            "greet: The robot needs to greet the person. The user DOES NOT provided his/her name", 
            "remember user: The robot needs to remember the name of the person that is taliking. This happens ONLY when the person introduce itself to the robot by saying his/her own name, that is the main difference against the action \"greet\", as an example the user might say: hello my name is David, hi I'm Joshua, etc."
            "goodbye: The robot needs to say goodbye to the person",
            "provide information: The robot needs to answer a question or explain a topic given by the user",
            "go to: The robot needs to move to an specific place. This action can be detonated by asking or demaning, examples: \"Go to the kitchen\", \"Go to the corner\", \"Move 5 meters\" ",
            "follow user: The robot needs to continuously following a person and not just \"go to\" towards them, \"come with me\" is an example of the user asking the robot to folow user", 
            "find object: The robot have to start moving around, in order to find the object or person asked. This action can be detonated by asking things like \"Where is the red ball?\" or demaning a search like \"Find a chair\", \"Where is David?\", \"Find Laura\" ", 
            "explore: The has to start moving around to create a map of the place, this action can be detonated by saying things like: \"start exploring\", \"create a map\" ", 
            "translate: The robot has to help the user to translate sentences or a conversation", 
            "other: If any of the other actions does not fit, the robot has to classify it as other"]
        
        #self.api          = Client(host="http://192.168.0.106:11434")
        self.api          = Client(host="http://130.229.157.65:11434")
        self.system_promt = "You are an office assistant robot caled Ari. Be concise and helpful."

        # Publishers
        self.dictonary_pub = rospy.Publisher("/brain/intent",String,queue_size=10)

        # Subscribers
        #rospy.Subscriber("/humans/voices/anonymous_speaker/speech",LiveSpeech,self.asr_result)
        rospy.Subscriber("/audio/speech", AudioData, self.stt)
        rospy.Subscriber("/brain/state",String,self.update_brain_state)
        rospy.Subscriber("/brain/user_name",String, self.update_brain_person)

        # Action client of TTS Multilanguages
        self.ac_ttsm = SimpleActionClient('tts_multilanguage', TextToSpeechMultilanguageAction)
        self.ac_ttsm.wait_for_server()


        self.setup_tts()
        self.calibrate_api()

        rospy.loginfo("[LLM            ]:Initialized")
        self.tts_output("Ready to operate")
        rospy.sleep(3)     
        self.stt_listen   = True

    def setup_tts(self):
        self.tts_client = SimpleActionClient("/tts",TtsAction)
        self.tts_client.wait_for_server()

    def stt(self,msg):
        if not msg.data or not self.stt_listen:
            return
        # Open WAV file for writing
        else:
            self.stt_listen   = False
        
        with wave.open(self.stt_filename, 'wb') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.SAMPLE_WIDTH)
            wf.setframerate(self.SAMPLE_RATE)
            wf.writeframes(msg.data)  # msg.data is a bytes object
        #rospy.loginfo(f"Saved audio to {self.stt_filename}")

        result            = self.stt_model.transcribe(self.stt_filename)
        self.stt_result   = result["text"]
        self.stt_language = result["language"]

        rospy.loginfo(f"[LLM            ]:User said: {self.stt_result}")
        if (self.stt_result).lower() == "stop":
            self.tts_output("Stopping")
            self.listen = False
            self.publish_intent("stop","")
            return
        if (self.stt_result).lower()  == "start" or (self.stt_result).lower()  == "init":
            self.tts_output("Listening")
            self.listen = True
            return
        
        self.process_user_input(self.stt_result)


    def calibrate_api(self):
        rospy.loginfo("[LLM            ]:Initializing calibraton...")
        test_sentences = ["Hello","Find the red ball", "Hello, I am Joshua"]

        for msg in test_sentences:
            query = self.build_intent_query(msg)
            self.ask_ollama(query)

        rospy.loginfo("[LLM            ]:Calibraton done")

    def update_brain_state(self,msg):
        self.brain_state_data = msg.data
        self.brain_msg_time   = rospy.Time.now().to_sec()

    def update_brain_person(self,msg):
        self.brain_person_name = msg.data

    #def asr_result(self,msg):
    #    if not msg.final:
    #        return
    #    rospy.loginfo(f"[LLM            ]:User said: {msg.final}")
    #    if (msg.final).lower() == "stop":
    #        self.tts_output("Stopping")
    #        self.listen = False
    #        self.publish_intent("stop","")
    #        return
    #    if (msg.final).lower()  == "start" or (msg.final).lower()  == "init":
    #        self.tts_output("Listening")
    #        self.listen = True
    #        return
    #    if not self.listen:
    #        return
    #    
    #    self.process_user_input(msg.final)

    def wait_for_brain(self,actual_time):
        while (self.brain_state_data != "Idle" ) or (self.brain_state_data == "Idle" and (self.brain_msg_time - actual_time) < 2.2 ): 
            rospy.sleep(1.0)
            rospy.logdebug(f"[LLM            ]:Waiting for brain: {self.brain_state_data}")

        rospy.logdebug(f"[LLM            ]:State:{self.brain_state_data}, Diff: {self.brain_msg_time - actual_time}")

    def ask_ollama(self, msg, promt=""):
        completion = self.api.chat(
            model=self.model_ollama, 
            messages=[
            {"role":"system","content": self.system_promt + promt},
            {"role":"user","content": msg},
            ]
        )

        return completion.message.content
    
    
    def build_intent_query(self, user_input):
        
        msg = ["The robot needs to execute one of the following actions:", ' '.join(list(self.intents.keys())), 
               "The description of each action is the following: ", ' '.join(self.intents_description), 
               ".The user said:", str(user_input),"What action did the user expect from the robot?",
               ".RETURN ONLY the option that best matches from the list provided, DO NOT mention any of the other ones in the response.", 
               "Additionally If the intent is \"go to\" return \"go to:specified_place\", if it is \"translate\" return \"translate to:specified_language\"", 
               " if it is \"find object\" return \"find object:object_to_find\"",
               " if it is \"remember user\" return \"remember user:user_name\""]
        msg = ' '.join(msg)
        return msg
    
    def extract_intent(self, intent_ollama):
        # Check this, as sometimes it fails to return the first one.
        if "greet" in intent_ollama.split():
            return "greet"
        else:
            phrase_words = set(re.findall(r'\b\w+\b', intent_ollama.lower()))  # Tokenize phrase into words
            for intent in sorted(list(self.intents.keys()), key=len, reverse=True):  # Match longer intents first
                intent_words = set(intent.lower().split())  # Split intent into words
                if intent_words.issubset(phrase_words):  # Check if all intent words are in the phrase
                    return intent
            
        return "No match found"
    
    def process_intent(self, intent_ollama, intent_result, user_input):
        parameter = "unknown"
        response  = ""

        if self.intents[intent_result][0] and intent_result != "remember user":
            response = f"Initializing '{intent_result}' action."
        
        if self.intents[intent_result][1] and ":" in intent_ollama:
            parameter = intent_ollama.split(":")[-1].strip().lower().replace("the ","").replace("\"","").replace(".","")
            response  = response + f" Objective: {parameter}"

        if not self.intents[intent_result][0]:
            if intent_result == "provide information":
                response = self.ask_ollama(f"{user_input}. Response in 15 words")
            else:
                response = self.ask_ollama(f"{user_input}. Response in 15 words",f"Use my name in your response, my name is: {self.brain_person_name}")


        return parameter, response
    
    def publish_intent(self, intent, parameter):
        self.listen = False
        #intent_data = {"intent":intent,"input": parameter}
        #self.dictonary_pub.publish(json.dumps(intent_data))
        #rospy.loginfo(f"Published speech intent: {response}")

        #if intent not in {"greet","goodbye","provide informaton","translate"}:
        #    intent_data = {"intent":intent, "input":parameter}
        #    self.dictonary_pub.publish(json.dumps(intent_data))
        #    rospy.loginfo(f"Published intent: {intent} with parameter: {parameter}")

        intent_data = {"intent":intent,"input": parameter}
        self.dictonary_pub.publish(json.dumps(intent_data))
        rospy.logdebug(f"[LLM            ]:Published intent:{intent}, Input:{parameter}")

        self.wait_for_brain(rospy.Time.now().to_sec())

    def reject_message(self):
        return "I could not understand, please say it again."

    def tts_output(self,text):
        goal                 = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text    = text
        self.tts_client.send_goal(goal)

    
    def tts_multilanguage_output(self,text):
        tts = gTTS(text, self.stt_language)
        save_path = os.path.expanduser('/tmp/tts_audio.mp3')

        # Save the audio file
        tts.save(save_path)

        # Send audio to robot
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect("192.168.128.28", username="pal", password="pal")

        sftp = ssh.open_sftp()
        sftp.put("/tmp/tts_audio.mp3", "/tmp/tts_audio.mp3")
        sftp.close()
        ssh.close()

        # Send goal to TTS Multilanguage server to reproduce audio
        goal = TextToSpeechMultilanguageGoal()
        self.ac_ttsm.send_goal(goal)

    def process_user_input(self, user_input):
        query         = self.build_intent_query(user_input)
        intent_ollama = self.ask_ollama(query)
        #rospy.loginfo("Intent ollama:",intent_ollama)
        rospy.logdebug(self.string_header + intent_ollama)
        intent_result = self.extract_intent(intent_ollama)
        rospy.loginfo(self.string_header + intent_result)

        if intent_result in {"No match found","other"}:
            self.listen = False
            response = self.reject_message()
            if self.stt_language == "en":
                self.tts_output(response)
            else:
                self.tts_multilanguage_output(response)

        else:
            if intent_result in ["greet","goodbye"]:
                self.publish_intent(intent_result,"")

            parameter, response = self.process_intent(intent_ollama, intent_result, user_input)
            rospy.loginfo(f"[LLM            ]:Ollama response: {response}")
            self.listen = False
            if self.stt_language == "en":
                self.tts_output(response)
            else:
                self.tts_multilanguage_output(response)

            if intent_result not in ["greet","goodbye"]:
                self.publish_intent(intent_result, parameter)

        self.listen      = True
        self.stt_listen  = True
        return response
    
if __name__ == '__main__':
    try:
        ChatboxARI()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Chatbos: ROS Node interrupted.")
