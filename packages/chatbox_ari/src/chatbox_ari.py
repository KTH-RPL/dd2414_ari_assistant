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

        self.dictionary_pub = rospy.Publisher("/intent/detect",String,queue_size=10)
        self.language = "en_US"

        base_url = "https://api.aimlapi.com/v1"
        api_key = "00485dc019b64535886a2ce0eff23b84" 
        
        self.api = Client(
            host='http://192.168.0.106:11434',
            headers={'x-some-header': 'some-value'}
        )


        self.system_prompt = "You are a office assistant robot called ARI (not just an AI). Be concise and helpful, give short answers."

        # Subscribe to ASR topic
        #self.asr_sub = rospy.Subscriber('/humans/voices/anonymous_speaker/speech',LiveSpeech,self.asr_result)

        # Set up PAL Robotics TTS
        #self.tts_client = SimpleActionClient("/tts", TtsAction)
        #self.tts_client.wait_for_server()

        rospy.loginfo("OLLAMA ARI node ready!")
        self.run()

    # For testing porpuses
    def run(self):
        user_input = ""
        while user_input != "stop":
            user_input = input("Insert sentence: ")
            self.asr_result(user_input)



    def asr_result(self, msg):
        """ Recognize speech. """
        sentence = msg.final
        if not sentence:
            return
        
        rospy.loginfo(f"User said: {sentence}")
        # Query DeepSeek with the user input
        response = self.query_deepseek(sentence)

        # Output the response through TTS
        if response:
            self.tts_output(response)

    def query_deepseek(self, user_input):
        """ Query DeepSeek API for a response. """
        intents = ["greet","goodbye","follow user","provide information","find object","go to","explore","translate","other"]
        intents_description = ["greet: The robot needs to greet the person", 
                               "goodbye: The robot needs to say goodbye to the person", 
                               "provide information: The robot needs to answer a question or explain a topic given by the user",
                               "go to: The robot needs to move to an specific place. This action can be detonated by asking or demaning, examples: \"Go to the kitchen\", \"Go to the corner\", \"Move 5 meters\" ",
                               "follow user: The robot needs to continuously following a person and not just \"go to\" towards them", 
                               "find object: The robot have to start moving around, in order to find the object or person asked. This action can be detonated by asking things like \"Where is the red ball?\" or demaning a search like \"Find a chair\", \"Where is David?\", \"Find Laura\" ", 
                               "explore: The has to start moving around to create a map of the place, this action can be detonated by saying things like: \"start exploring\", \"create a map\" ", 
                               "translate: The robot has to help the user to translate sentences or a conversation", 
                               "other: If any of the other actions does not fit, the robot has to classify it as other"]

        model_ollama = "mistral:latest" #llama3.2:latest, mistral:latest, deepseek-r1:latest
        try:
            # Generate a response using the transformer pipeline
            completion = self.api.chat(
                model=model_ollama, 
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": "The robot needs to execute one of the following actions: "+str(intents) + 
                     "The description of each action is the following: " + str(intents_description) + "."
                     "The user said: "+str(user_input) 
                     + "What action did the user expect from the robot?"+ " .RETURN ONLY the option that best matches from the list provided, DO NOT mention any of the other ones in the response." 
                     + "Additionally If the intent is \"go to\" return \"go to:specified_place\", if it is \"translate\" return \"translate to:specified_language\"" +
                     " if it is \"find object\" return \"find object:object_to_find\""},
                    ] 
            )
            
            intent_ollama  = completion.message.content
            rospy.loginfo(f"intent OLLAMA: {intent_ollama}")

            phrase_words = set(re.findall(r'\b\w+\b', intent_ollama.lower()))  # Tokenize phrase into words
            for intent in sorted(intents, key=len, reverse=True):  # Match longer intents first
                intent_words = set(intent.lower().split())  # Split intent into words
                if intent_words.issubset(phrase_words):  # Check if all intent words are in the phrase
                    intent_result = intent
                    break
                else:
                    intent_result = "No match found"

            if intent_result == "No match found" or intent_result == "other":
                ## ASK USER TO SAY AGAIN 
                response = "I could not understand, please say it again"


            else:
                rospy.loginfo(f"intent: {intent_result}")        
                parameter = ""
                if ":" in intent_ollama and intent_result in ["go to","follow user","find object","explore","translate"]:
                    if intent_result != "follow user" and intent_result != "explore":
                        parameter = intent_ollama.split(": ")
                        parameter = parameter[1]
                        if "the" in parameter:
                            parameter = parameter.replace("the ","")

                    response = "Intializing \"" + intent_result + "\" action."
                    if parameter != "":
                        response = response + " Objective: " + parameter

                else:
                    completion = self.api.chat(
                        model=model_ollama,
                        messages=[
                            {"role": "system", "content": self.system_prompt + ". Response with no more than 15 words"},
                            {"role": "user", "content": user_input},
                            ]
                    )
                    response = completion.message.content
                    parameter = ""
                
                #Publish the intent
                intent_dictionary = {"intent":intent_result,"input":parameter}
                json_string = json.dumps(intent_dictionary)
                self.dictionary_pub.publish(json_string)        
                
                rospy.loginfo(f"DeepSeek Response: {response}")
            
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
