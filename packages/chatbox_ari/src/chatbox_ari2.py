#!/usr/bin/env python3
import rospy
import json
import re

from ollama import Client
from actionlib import SimpleActionClient
from std_msgs.msg import String
from hri_msgs.msg import LiveSpeech
from pal_interaction_msgs.msg import TtsAction, TtsGoal

class ChatboxARI:
    def __init__(self):
        rospy.init_node("chatbox_ari")
        self.rate   = rospy.Rate(1)
        self.listen = True

        self.stop              = False
        self.brain_state_data  = "Busy"
        self.brain_person_name = "unknown"
        self.brain_msg_time    = rospy.Time.now().to_sec()
        self.language          = "en_US"
        self.model_ollama      = "mistral:latest"

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
        
        self.intents_description  = (
            f"greet: The robot needs to greet the person. The user DOES NOT provided his/her name.\n", 
            f"remember user: The robot needs to remember the name of the person that is taliking. This happens ONLY when the person introduce itself to the robot by saying his/her own name, that is the main difference against the action \"greet\", as an example the user might say: hello my name is David, hi I'm Joshua, etc.\n"
            f"goodbye: The robot needs to say goodbye to the person.\n",
            f"provide information: The robot needs to answer a question or explain a topic given by the user.\n",
            f"go to: The robot needs to move to an specific place. This action can be detonated by asking or demaning, examples: \"Go to the kitchen\", \"Go to the corner\", \"Move 5 meters\".\n",
            f"follow user: The robot needs to continuously following a person and not just \"go to\" towards them, \"come with me\" is an example of the user asking the robot to folow user.\n", 
            f"find object: The robot have to start moving around, in order to find the object or person asked. This action can be detonated by asking things like \"Where is the red ball?\" or demaning a search like \"Find a chair\", \"Where is David?\", \"Find Laura\".\n", 
            f"explore: The has to start moving around to create a map of the place, this action can be detonated by saying things like: \"start exploring\", \"create a map\". \n", 
            f"translate: The robot has to help the user to translate sentences or a conversation.\n", 
            f"other: If any of the other actions does not fit, the robot has to classify it as other.\n")
        
        self.api          = Client(host="http://192.168.0.106:11434")
        self.system_promt = "You are an office assistant robot caled Ari. Be concise and helpful."

        # Publishers
        self.dictonary_pub = rospy.Publisher("/brain/intent",String,queue_size=10)

        # Subscribers
        rospy.Subscriber("/humans/voices/anonymous_speaker/speech",LiveSpeech,self.asr_result)
        rospy.Subscriber("/brain/state",String,self.update_brain_state)
        rospy.Subscriber("/brain/user_name",String, self.update_brain_person)

        self.setup_tts()
        self.calibrate_api()

        rospy.loginfo("Chatbox ARI node ready")
        self.tts_output("Ready to operate")        

    def setup_tts(self):
        self.tts_client = SimpleActionClient("/tts",TtsAction)
        self.tts_client.wait_for_server()

    def calibrate_api(self):
        rospy.loginfo("Initializing calibraton...")
        test_sentences = ["Hello","Find the red ball", "Hello, I am Joshua"]

        for msg in test_sentences:
            query = self.build_intent_query(msg)
            self.ask_ollama(query)

        rospy.loginfo("Calibraton done")

    def update_brain_state(self,msg):
        self.brain_state_data = msg.data
        self.brain_msg_time   = rospy.Time.now().to_sec()

    def update_brain_person(self,msg):
        self.brain_person_name = msg.data

    def asr_result(self,msg):
        if not msg.final or not self.listen:
            return
        rospy.loginfo(f"User said: {msg.final}")
        if msg.final.lower() == "stop":
            self.listen = False
            return
        if msg.final.lower()  == "start":
            self.listen = True
            return
        
        response = self.process_user_input(msg.final)

    def wait_for_brain(self,actual_time):
        while (self.brain_state_data != "Idle" ) or (self.brain_state_data == "Idle" and (self.brain_msg_time - actual_time) < 2.1 ): 
            rospy.sleep(1.0)
            rospy.loginfo(f"Waiting for brain: {self.brain_state_data}")

        rospy.loginfo(f"State:{self.brain_state_data}, Diff: {self.brain_msg_time - actual_time}")

    def ask_ollama(self, msg, promt=""):
        return self.api.chat(model=self.model_ollama, messages=[
            {"role":"system","content":self.system_promt + promt},
            {"role":"user","content":msg}
        ]).message.content
    
    def build_intent_query(self, user_input):
        descriptions = "".join(self.intents_description)
        return (
            f"The robot needs to execute one of the following actions: {', '.join(self.intents.keys())}.\n"
            f"Descriptions: {descriptions}."
            f"The user said: {user_input}.\n"
            "What action did the user expect from the robot?\n"
            "Return only the best matching option, formatted as following:\n"
            "If greet: return greet\n"
            "If goodbye: return goodbye\n"
            "If remember user: return \"remember user:user_name\"\n"
            "If provide information: return provide information\n"
            "If go to: return \"go to:specified_place\"\n"
            "If follow user: return follow user\n"
            "If find object: return \"find object:object_to_find\"\n"
            "If explore: return \n"
            "If translate: return \"translate to:specified_language\"\n"
            "If other: return other\n"
        )
    
    def extract_intent(self, intent_ollama):
        for intent in sorted(self.intents.keys(),key=len,reverse=True):
            if intent in intent_ollama:
                return intent
        return "No match found"
    
    def process_intent(self, intent_ollama, intent_result, user_input):
        parameter = "unknown"
        response  = ""

        if self.intents[intent_result][0] and intent_result != "remember user":
            response = f"Initializing '{intent_result}' action."
        
        if self.intents[intent_result][1] and ":" in intent_ollama:
            parameter = intent_ollama.split(":")[-1].strip().lower().replace("the ","")
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
        rospy.loginfo(f"Published intent:{intent}, Input:{parameter}")

        self.wait_for_brain(rospy.Time.now().to_sec())

        self.listen = True

    def reject_message(self):
        return "I could not understand, please say it again."

    def tts_output(self,text):
        goal                 = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text    = text
        self.tts_client.send_goal(goal)

    def process_user_input(self, user_input):
        query         = self.build_intent_query(user_input)
        intent_ollama = self.ask_ollama(query)
        rospy.loginfo("Intent ollama:",intent_ollama)
        intent_result = self.extract_intent(intent_ollama)

        if intent_result in {"No match found","other"}:
            return self.reject_message()
        
        if intent_result in ["greet","goodbye"]:
            self.publish_intent(intent_result,"")

        parameter, response = self.process_intent(intent_ollama, intent_result, user_input)
        self.tts_output(response)

        if intent_result not in ["greet","goodbye"]:
            self.publish_intent(intent_result, parameter)

        return response
    
if __name__ == '__main__':
    try:
        ChatboxARI()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Chatbos: ROS Node interrupted.")
