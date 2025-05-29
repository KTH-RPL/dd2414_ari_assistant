#!/usr/bin/env python3
import re
import json
import rospy
import whisper
import dd2414_text_speech.msg as tts

from pydub import AudioSegment
from ollama import Client
from actionlib import SimpleActionClient
from std_msgs.msg import String
from deep_translator import GoogleTranslator
from pal_interaction_msgs.msg import TtsAction, TtsGoal

class ChatboxARI:
    def __init__(self):
        self.rate              = rospy.Rate(1)
        self.listen            = False
        self.stop              = False
        self.ready_to_process  = True
        self.data_dic          = None
        self.string_header     = "[LLM            ]:"
        self.languages         = ["en","es","de","fr","sv"]
        self.languages         = {"en":"English","es":"Spanish","de":"German","fr":"French","sv":"Swedish","jp":"Japanese"}

        self.ari_speeking      = ""
        self.ari_translating   = ""
        self.model_ollama      = "mistral:latest"
        self.verbose           = rospy.get_param("/verbose",False)
        self.stt_result        = ""
        self.stt_language      = "en"
        self.source_language   = ""

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
            "greet: The robot needs to greet the person. This action is detonated when the user greets and DOES NOT provided his/her name in the sentence", 
            "remember user: The robot needs to remember the name of the person that is taliking. This happens ONLY when the person introduce itself to the robot by saying his/her own name, that is the main difference against the action \"greet\", as an example the user might say: hello my name is David, hi I'm Joshua, etc."
            "goodbye: The robot needs to say goodbye to the person",
            "provide information: The robot needs to answer a question or explain a topic asked by the user",
            "go to: The robot needs to move to an specific place. This action can be detonated by asking or demaning, examples: \"Go to the kitchen\", \"Go to the corner\", \"Move 5 meters\" ",
            "follow user: The robot needs to continuously following a person and not just \"go to\" towards them, \"come with me\" is an example of the user asking the robot to folow user", 
            "find object: The robot have to start moving around, in order to find the object or person asked. This action can be detonated by asking things like \"Where is the red ball?\" or demaning a search like \"Find a chair\", \"Where is David?\", \"Find Laura\" ", 
            "explore: The has to start moving around to create a map of the place, this action can be detonated by saying things like: \"start exploring\", \"create a map\" ", 
            "translate: The robot has to help the user to translate sentences or a conversation", 
            "other: If any of the other actions does not fit, the robot has to classify it as other"]

        self.intents_des = [
        {
            "name": "greet",
            "description": "The user greets the robot without mentioning their name.",
            "examples": ["Hi", "Hello there", "Good morning", "Hey"],
            "note": "If the user includes their name, classify as 'remember user' instead."
        },
        {
            "name": "remember user",
            "description": "The user introduces themselves by saying their name.",
            "examples": ["Hi, my name is David", "I'm Sarah", "Hello, I am Joshua"],
            "note": "Must contain user's name. Distinct from a simple 'greet'."
        },
        {
            "name": "goodbye",
            "description": "The user ends the interaction by saying goodbye.",
            "examples": ["Bye", "Goodbye", "See you later", "Catch you later"]
        },
        {
            "name": "provide information",
            "description": "The user asks a question or requests an explanation about a topic.",
            "examples": ["What is photosynthesis?", "Can you explain how a battery works?", "Tell me about the weather"]
        },
        {
            "name": "go to",
            "description": "The user instructs the robot to move to a specific location or direction.",
            "examples": ["Go to the kitchen", "Move forward 3 meters", "Go to the door", "Head to the living room", "Go to Laura", "Go to David"]
        },
        {
            "name": "follow user",
            "description": "The user asks the robot to follow them continuously, not just go to a location.",
            "examples": ["Follow me", "Come with me", "Stay with me"],
            "note": "Involves continuous movement with the user, unlike 'go to'."
        },
        {
            "name": "find object",
            "description": "The user asks the robot to locate a specific object or person.",
            "examples": ["Where is the red ball?", "Find Laura", "Search for a chair", "Where is David?"],
            "note": "Triggers search behavior to find something or someone."
        },
        {
            "name": "explore",
            "description": "The user instructs the robot to explore the environment and map the area.",
            "examples": ["Start exploring", "Create a map", "Explore the house", "Scan the environment"]
        },
        {
            "name": "translate",
            "description": "The user asks the robot to translate words, phrases, or conversations.",
            "examples": ["Translate this to Spanish", "How do you say apple in French?", "Can you translate this sentence?"]
        },
        {
            "name": "other",
            "description": "The request does not fit any defined intent categories.",
            "examples": ["Tell me a joke", "What's your favorite color?", "Sing a song"],
            "note": "Fallback for out-of-scope or unclear requests."
        }]
        
        self.intent_json = json.dumps(self.intents_des, indent=2)
        
        #self.api          = Client(host="http://130.229.183.186:11434")
        self.api          = Client(host="http://localhost:11434")
        self.system_promt = "You are an office assistant robot caled Ari. Be concise and helpful."

        # Publishers
        self.dictonary_pub = rospy.Publisher("/brain/intent",String,queue_size=10)

        # Subscribers
        #rospy.Subscriber("/humans/voices/anonymous_speaker/speech",LiveSpeech,self.asr_result)
        rospy.Subscriber("/translate_conversation",String,self.ari_translating_state)
        rospy.Subscriber("/tts/ARI_speeking",String,self.ari_speeking_state)
        rospy.Subscriber("/stt/transcript",String,self.stt)

        self.setup_tts()
        self.calibrate_api()

        rospy.loginfo("[LLM            ]:Initialized")
        self.tts_output("Ready to operate")

        rospy.sleep(3) 
        self.listen   = True

    def setup_tts(self):
        self.tts_client = SimpleActionClient("/tts",TtsAction)
        self.tts_client.wait_for_server()
        rospy.loginfo("[LLM            ]:TTS UP")

    def tts_output(self,text):
        if self.ari_translating != "translating":
            self.listen          = False
            goal                 = TtsGoal()
            goal.rawtext.lang_id = "en_US"
            goal.rawtext.text    = text
            self.tts_client.send_goal_and_wait(goal)
            self.tts_client.wait_for_result()
            rospy.sleep(1.7)

    def ari_translating_state(self,msg):
        self.ari_translating = msg.data

    def ari_speeking_state(self,msg):
        self.ari_speeking = msg.data

    def reject_message(self):
        self.tts_output("I could not understand, please say it again.")

    def build_intent_query(self, user_input):
        
        msg = ["The robot needs to execute one of the following actions:", ' '.join(list(self.intents.keys())), 
               "The description of each action is the following: ", ' '.join(self.intents_description), 
               ".The user said:", str(user_input),"What action did the user expect from the robot?",
               ".RETURN ONLY the option that best matches from the list provided, DO NOT mention any of the other ones in the response.", 
               "Additionally If the intent is \"go to\" return \"go to:specified_place\", if it is \"translate\" return \"translate to:specified_language\"", 
               " if it is \"find object\" return \"find object:object_to_find\"",
               " if it is \"remember user\" return \"remember user:user_name\""]
        #msg = [f"Based on the following intent schema, classify the user's message into one of the intent names. Intent schema: {self.intent_json} User input: {user_input} What action did the user expect from the robot? RETURN ONLY the option that best matches. Additionally If the intent is \"go to\" return \"go to:specified_objective\", if it is \"translate\" return \"translate to:specified_language\", if it is \"find object\" return \"find object:object_to_find\", if it is \"remember user\" return \"remember user:user_name\""]

        msg = f"""
        You are a classifier that determines both the intent and key details (parameters) from a user's message. 
        Based on the following intent schema, classify the user's input and extract the relevant parameter if applicable.
        Intent schema: {self.intent_json}
        Instructions:
        - For intent "go to", return in the format: go to:specified_place
        - For intent "translate", return in the format: translate to:specified_language
        - For intent "find object", return in the format: find object:object_to_find
        - For intent "remember user", return in the format: remember user:user_name
        - For all other intents, return just the intent name
        User input: "{user_input}"
        Respond with only one line in the correct format.
        """
        #msg = ' '.join(msg)

        return msg

    def ask_ollama(self, msg, promt=""):
        completion = self.api.chat(
            model=self.model_ollama, 
            messages=[
            {"role":"system","content": self.system_promt + promt},
            {"role":"user","content": msg},
            ]
        )

        return completion.message.content

    def calibrate_api(self):
        rospy.loginfo("[LLM            ]:Initializing calibraton...")
        test_sentences = ["Hello","Find the red ball", "Hello, I am Joshua"]

        for msg in test_sentences:
            query = self.build_intent_query(msg)
            self.ask_ollama(query)

        rospy.loginfo("[LLM            ]:Calibraton done")

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
    
    
    def publish_intent(self, intent, parameter, response):
        intent_data = {"intent":intent,"input":parameter,"phrase":response,"language":self.stt_language}
        self.dictonary_pub.publish(json.dumps(intent_data))
        rospy.loginfo(f"[LLM            ]:Published intent:{intent}, Input:{parameter}, Phrase:{response}, Lang:{self.stt_language}")

    def stt(self,msg):
        if not msg.data:
            return 

        self.data_dic = msg.data
        
        try:
            if self.listen and self.data_dic != None and self.ari_speeking != "speaking" :
                rospy.loginfo(self.data_dic)
            #self.run_stt()
            #self.ready_to_process = True
            
                self.data_dic = json.loads(self.data_dic)
                #rospy.loginfo("Listen:",self.listen,"Process:",self.ready_to_process,"Data Dic:",self.data_dic)
            else:
                
                self.data_dic = None
                return
        except Exception as e:
            rospy.logerr(f"Invalid data:{e}")
            return
        
    def run_stt(self):
        rospy.loginfo(f"Listen: {self.listen}, Process: {self.ready_to_process}, Stop: {self.stop}, Speeking:{self.ari_speeking}, Translating:{self.ari_translating}, Data:{self.data_dic}")

        if self.ari_speeking == "speaking" or not self.listen or self.data_dic==None: #or self.ari_translating=="translating":
            return
        
        if isinstance(self.data_dic, str):
            self.data_dic = json.loads(self.data_dic)

        self.listen= False
        self.ready_to_process = True

        self.stt_result      = self.data_dic["translation"].replace(".","")
        self.stt_language    = self.data_dic["language"]
        self.source_language = self.data_dic["language"]

        a = len(self.stt_result.split())
        rospy.loginfo(f"{self.stt_result},{a}")

        if "stop" in (self.stt_result).lower() and len(self.stt_result.split())<2:
            self.stop = True 
            self.ready_to_process = False
            self.listen = False
            self.publish_intent("stop","","Stopping")
            self.tts_output("Stopping")

        if "start" in (self.stt_result).lower()  or "continue" in ((self.stt_result).lower()).split() and len(self.stt_result.split(" "))<2:
            self.tts_output("Listening")
            self.stop = False
            self.listen = True
            self.ready_to_process = False

        if self.ari_translating=="translating":
            return
        
        if self.stop:
            self.listen = True
            return

        if not self.stt_language in list(self.languages.keys()):
            self.reject_message()
            self.listen = True
            self.ready_to_process = False
        
        if self.ready_to_process:
            rospy.loginfo(f"[LLM            ]:User said: {self.stt_result}, {self.stt_language}")
            self.process_user_input(self.stt_result,self.stt_language)
            self.listen           = True
            self.ready_to_process = False

        self.data_dic = None
            

    def process_intent(self, intent_ollama, intent_result, user_input,lang):
        parameter = "unknown"
        response  = ""
        
        if self.intents[intent_result][1] and ":" in intent_ollama:
            parameter = intent_ollama.split(":")[-1].strip().lower().replace("the ","").replace("\"","").replace(".","")
            aux = parameter.split()
            #rospy.loginfo(aux)
            #rospy.loginfo(len(aux))
            if len(aux) > 3 or "specified" in aux:
                return "", ""
            
            response  = response + f" Objective: {parameter}"
        
        if self.intents[intent_result][0] and intent_result != "remember user":
            response = f"Initializing {intent_result} action."
            if intent_result == "translate":
                rospy.loginfo(lang)
                rospy.loginfo(self.source_language)
                parameter = parameter + "," + str(lang)
            self.stt_language = "en"


        if not self.intents[intent_result][0]:
            if intent_result == "provide information" or intent_result == "remember user":
                response = self.ask_ollama(f"{user_input}. Answer in a short phrase, less than 27 words and JUST in {self.languages[self.stt_language]} language, do not include the translation in english")
                #if self.stt_language != 'en':
                #    response = GoogleTranslator(source='en', target=self.stt_language).translate(response)
            else:
                response = user_input

        parameter = parameter.lower()
        return parameter, response

    def process_user_input(self, user_input, language):
        lang = language
        if len(user_input) < 3:
            return

        query         = self.build_intent_query(user_input)
        intent_ollama = self.ask_ollama(query)
    
        rospy.logdebug(self.string_header + intent_ollama)
        intent_result = self.extract_intent(intent_ollama)
        rospy.loginfo(self.string_header + intent_result)

        if intent_result in {"No match found","other"}:
            self.reject_message()

        else:

            parameter, response = self.process_intent(intent_ollama, intent_result, user_input,lang)
            rospy.loginfo(len(response.split(" ")))
            if parameter == "" and response == "" or len(response.split(" ")) > 27:
                self.reject_message()
                return 
            
            #rospy.loginfo(f"[LLM            ]:Ollama response: {response}")
            #rospy.loginfo(f"STT language: {self.stt_language}")
            
            self.publish_intent(intent_result,parameter,response)

        return
    
if __name__ == '__main__':

    try:
        rospy.init_node("chatbox_ari",log_level=rospy.INFO)
        rate =rospy.Rate(1)
        node=ChatboxARI()
        while not rospy.is_shutdown():
            node.run_stt()
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("Chatbos: ROS Node interrupted.")

    

