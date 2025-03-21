#!/usr/bin/env python3
import numpy
import rospy
import json
import requests
import re
from std_msgs.msg import String
from actionlib import SimpleActionClient
from hri_msgs.msg import LiveSpeech
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from ollama import Client

class ChatboxARI:
    def __init__(self):
        rospy.init_node("chatbox_ari")
        self.rate = rospy.Rate(1)
        self.listen = True

        self.brain_state_data = "Busy"
        self.intents_action_split = {"greet":[False,False],"remember user":[False,True],"goodbye":[False,False],"follow user":[True,False],"provide information":[False,False],"find object":[True,True],"go to":[True,True],"explore":[True,False],"translate":[True,True],"other":False}
        self.intents_description  = [
            "greet: The robot needs to greet the person. The user does not provided his/her name", 
            "remember user: The robot needs to remember the name of the person that is taliking. This happens ALWAYS when the person introduce itself to the robot and say his/her name, that is the main difference against the action \"greet\" as an example the user might say: hello my name is David, hi I'm Joshua, hello Ari I am Katia, etc."
            "goodbye: The robot needs to say goodbye to the person",
            "provide information: The robot needs to answer a question or explain a topic given by the user",
            "go to: The robot needs to move to an specific place. This action can be detonated by asking or demaning, examples: \"Go to the kitchen\", \"Go to the corner\", \"Move 5 meters\" ",
            "follow user: The robot needs to continuously following a person and not just \"go to\" towards them, \"come with me\" is an example of the user asking the robot to folow user", 
            "find object: The robot have to start moving around, in order to find the object or person asked. This action can be detonated by asking things like \"Where is the red ball?\" or demaning a search like \"Find a chair\", \"Where is David?\", \"Find Laura\" ", 
            "explore: The has to start moving around to create a map of the place, this action can be detonated by saying things like: \"start exploring\", \"create a map\" ", 
            "translate: The robot has to help the user to translate sentences or a conversation", 
            "other: If any of the other actions does not fit, the robot has to classify it as other"]

        self.model_ollama = "mistral:latest" #llama3.2:latest, mistral:latest, deepseek-r1:latest

        self.dictionary_pub = rospy.Publisher("/brain/intent",String,queue_size=10)
        self.language = "en_US"

        base_url = "https://api.aimlapi.com/v1"
        api_key = "00485dc019b64535886a2ce0eff23b84" 
        
        self.api = Client(
            host='http://192.168.0.106:11434',
            headers={'x-some-header': 'some-value'}
        )

        self.system_prompt = "You are a office assistant robot called Ari (not just an AI). Be concise and helpful, give short answers."

        # API Calibration
        rospy.loginfo("INITIALIZING CALIBRATION")
        for initial_msg in ["Hello","Find the red ball","Hello, I am Joshua"]:
            msg = ["The robot needs to execute one of the following actions:", ' '.join(list(self.intents_action_split.keys())), "The description of each action is the following: ", ' '.join(self.intents_description), ".The user said:", initial_msg,"What action did the user expect from the robot?",
                ".RETURN ONLY the option that best matches from the list provided, DO NOT mention any of the other ones in the response.", 
                "Additionally If the intent is \"go to\" return \"go to:specified_place\", if it is \"translate\" return \"translate to:specified_language\"", 
                " if it is \"find object\" return \"find object:object_to_find\"",
                " if it is \"remember user\" return \"remember user:user_name\""]
            msg = ' '.join(msg)
            _ = self.ask_ollama("",msg)
        rospy.loginfo("CALIBRATION DONE")

        # Subscribe to ASR topic
        self.asr_sub = rospy.Subscriber('/humans/voices/anonymous_speaker/speech',LiveSpeech,self.asr_result)

        # Subscribe to brain state topic
        self.brain_sub = rospy.Subscriber('/brain/state',String,self.brain_state)

        # Set up PAL Robotics TTS
        self.tts_client = SimpleActionClient("/tts", TtsAction)
        self.tts_client.wait_for_server()

        rospy.loginfo("OLLAMA ARI node ready!")
        self.tts_output("Ready to operate")
        #self.run()

    # For testing porpuses
    #def run(self):
    #    user_input = ""
    #    while user_input != "stop":
    #        user_input = input("Insert sentence: ")
    #        self.asr_result(user_input)

    def brain_state(self,state):
        self.brain_state_data = state.data

    def asr_result(self, msg):
        """ Recognize speech. """
        sentence = msg.final
        #sentence = msg
        if not sentence:
            return
        
        rospy.loginfo(f"User said: {sentence}")
        # Query DeepSeek with the user input
        if self.listen:
            response = self.query_deepseek(sentence)

        # Output the response through TTS
        #if response:
        #    self.tts_output(response)

    def ask_ollama(self,promt,msg):
        completion = self.api.chat(
            model=self.model_ollama,
            messages=[
                {"role": "system", "content": self.system_prompt + promt},
                {"role": "user", "content": msg},
                ]
        )
        
        return completion.message.content

    def process_intent(self,intent_ollama,intent_result,user_input):
        parameter = "unknown"
        response  = ""
        promt     = ". Response with no more than 15 words."

        if self.intents_action_split[intent_result][0]:
            response = "Intializing \"" + intent_result + "\" action."
        
        #if intent_result in ["go to","follow user","find object","explore","translate","remember user"]:
        if self.intents_action_split[intent_result][1] and ":" in intent_ollama:
            if ": " in intent_ollama: 
                parameter = intent_ollama.split(": ")
            else:
                parameter = intent_ollama.split(":")
                        
            parameter = parameter[1]

            if len(parameter.split()) > 2:
                 return "", self.reject_msg()

            if "the" in parameter:
                parameter = parameter.replace("the ","")

            response = response + " Objective: " + parameter

        if not self.intents_action_split[intent_result][0]:
            if intent_result == "remember user":
                promt = promt + "Use my name in your response, my name is:" + parameter

            response = self.ask_ollama(promt,user_input)
            
        return parameter, response
    
    def reject_msg(self):

        return "I could not understand, please say it again"
    
    def pub_dictionary(self):
        pass


    def query_deepseek(self, user_input):
        """ Query DeepSeek API for a response. """
                
        try:
            # Generate a response using the transformer pipeline
            msg = ["The robot needs to execute one of the following actions:", ' '.join(list(self.intents_action_split.keys())), "The description of each action is the following: ", ' '.join(self.intents_description), ".The user said:", str(user_input),"What action did the user expect from the robot?",
            ".RETURN ONLY the option that best matches from the list provided, DO NOT mention any of the other ones in the response.", 
            "Additionally If the intent is \"go to\" return \"go to:specified_place\", if it is \"translate\" return \"translate to:specified_language\"", 
            " if it is \"find object\" return \"find object:object_to_find\"",
            " if it is \"remember user\" return \"remember user:user_name\""]
            msg = ' '.join(msg)
            
            intent_ollama  = self.ask_ollama("",msg)
            rospy.loginfo(f"intent OLLAMA: {intent_ollama}")

            phrase_words = set(re.findall(r'\b\w+\b', intent_ollama.lower()))  # Tokenize phrase into words
            for intent in sorted(list(self.intents_action_split.keys()), key=len, reverse=True):  # Match longer intents first
                intent_words = set(intent.lower().split())  # Split intent into words
                if intent_words.issubset(phrase_words):  # Check if all intent words are in the phrase
                    intent_result = intent
                    break
                else:
                    intent_result = "No match found"

            rospy.loginfo(f"intent: {intent_result}") 
            if intent_result == "No match found" or intent_result == "other":
                ## ASK USER TO SAY AGAIN 
                response = self.reject_msg()

            else:
                parameter, response = self.process_intent(intent_ollama,intent_result,user_input)
                if response == "":
                    intent_result =""
                
                #Publish the speech
                self.listen = False
                intent_dictionary = {"intent":"speech","input":response}
                json_string = json.dumps(intent_dictionary)
                self.dictionary_pub.publish(json_string)        
                print(json_string)
                rospy.loginfo(f"DeepSeek Response: {response}")
                
                while self.brain_state_data != "Idle":
                    rospy.loginfo("Waiting for brain")
                    self.rate.sleep()

                # Waiting for brain
                if intent_result != "greet" and intent_result != "goodbye" and intent_result != "provide information":
                
                    #Publish the intent
                    intent_dictionary = {"intent":intent_result,"input":parameter}
                    json_string = json.dumps(intent_dictionary)
                    self.dictionary_pub.publish(json_string)        
                    print(json_string)
                    #rospy.loginfo(f"DeepSeek Response: {response}")
                
                self.listen = True
            
            return response
            
        except Exception as e:
            rospy.logerr(f"Error generating response: {e}")
            return "Sorry, I couldn't process your request."

    def tts_output(self, response):
        """ Output the response through TTS. """
        goal = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text = response
        self.tts_client.send_goal_and_wait(goal)

if __name__ == '__main__':
    try:
        chatbox_ari = ChatboxARI()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Node interrupted.")


 #completion = self.api.chat(
            #    model=self.model_ollama, 
            #    messages=[
            #        {"role": "system", "content": self.system_prompt},
            #        {"role": "user", "content": "The robot needs to execute one of the following actions: "+str(self.intents) + 
            #         "The description of each action is the following: " + str(intents_description) + "."
            #         "The user said: "+str(user_input) 
            #         + "What action did the user expect from the robot?"+ " .RETURN ONLY the option that best matches from the list provided, DO NOT mention any of the other ones in the response." 
            #         + "Additionally If the intent is \"go to\" return \"go to:specified_place\", if it is \"translate\" return \"translate to:specified_language\"" 
            #         + " if it is \"find object\" return \"find object:object_to_find\""
            #         + " if it is \"remember user\" return \"remember user:user_name\""},
            #        ] 
            #)
