#!/usr/bin/env python3
import numpy
import rospy
import requests
import re
from actionlib import SimpleActionClient
from hri_msgs.msg import LiveSpeech
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from ollama import Client

class ChatboxARI:
    def __init__(self):
        rospy.init_node("chatbox_ari")
        self.language = "en_US"

        base_url = "https://api.aimlapi.com/v1"
        api_key = "00485dc019b64535886a2ce0eff23b84" 
        
        self.api = Client(
            host='http://192.168.0.106:11434',
            headers={'x-some-header': 'some-value'}
        )


        self.system_prompt = "You are a office assistant robot called ARI. Be concise and helpful, give short answers."

        # Subscribe to ASR topic
        self.asr_sub = rospy.Subscriber(
            '/humans/voices/anonymous_speaker/speech',
            LiveSpeech,
            self.asr_result
        )

        # Set up PAL Robotics TTS
        self.tts_client = SimpleActionClient("/tts", TtsAction)
        self.tts_client.wait_for_server()

        rospy.loginfo("OLLAMA ARI node ready!")

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
        intends = ["greet","goodbye","small talk","follow user","provide information","find object","move to","explore","translate","other"]
        model_ollama = "mistral:latest" #llama3.2:latest, mistral:latest, deepseek-r1:latest
        try:
            # Generate a response using the transformer pipeline
            completion = self.api.chat(
                model=model_ollama, 
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": "The robot needs to execute one of the following actions: "+str(intends) + "The user said: "+str(user_input) 
                     + "What intend did the user expect from the robot"+ " .RETURN ONLY the option that best matches from the list provided, DO NOT mention any of the other ones in the response." 
                     + " .Additionally If the intend is \"move to\" return \"move to <place>\", if it is \"translate\" return \"translate to <language>\"" +
                     " if it is \"find object\" return \"find object <object_name>\"" + " .Important to know, \"follow user\" refeers to cotinuously following a person and not just \"move to\" towards them, they are differnet intends." },
                    ] 
            )
            
            intend  = completion.message.content
            rospy.loginfo(f"Intend OLLAMA: {intend}")

            phrase_words = set(re.findall(r'\b\w+\b', intend.lower()))  # Tokenize phrase into words
            for intend in sorted(intends, key=len, reverse=True):  # Match longer intents first
                intend_words = set(intend.lower().split())  # Split intent into words
                if intend_words.issubset(phrase_words):  # Check if all intent words are in the phrase
                    intend_result = intend
                    break
                else:
                    intend_result = "No match found"

            if intend_result == "No match found" or intend_result == "other":
                ## ASK USER TO SAY AGAIN 
                response = "I could not understand, please say it again"
            else:        
                #ACCORDING TO THE INTEND CALL THE CORRESPONDIG FUNCTION
                rospy.loginfo(f"Intend: {intend_result}")
                if intend_result == "greet" or intend_result == "goodbye":
                    completion = self.api.chat(
                        model=model_ollama,
                        messages=[
                            {"role": "system", "content": self.system_prompt + ". Response with no more than 15 words"},
                            {"role": "user", "content": user_input},
                            ]
                    )
                    response = completion.message.content
                    rospy.loginfo(f"DeepSeek Response: {response}")

                elif intend_result == "small talk" or intend_result == "provide information":
                    completion = self.api.chat(
                        model=model_ollama,
                        messages=[
                            {"role": "system", "content": self.system_prompt},
                            {"role": "user", "content": user_input},
                            ]
                    )
                    response = completion.message.content
                    rospy.loginfo(f"DeepSeek Response: {response}")
                ## ADD REMAINING FUCNTIONS HERE
                else:
                    completion = self.api.chat(
                        model=model_ollama,
                        messages=[
                            {"role": "system", "content": self.system_prompt + ". Response with no more than 15 words"},
                            {"role": "user", "content": user_input},
                            ]
                    )
                    response = completion.message.content
                    response = "Hi"
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
