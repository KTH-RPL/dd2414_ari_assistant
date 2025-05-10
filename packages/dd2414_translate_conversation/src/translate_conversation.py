#!/usr/bin/python3
import rospy
import json
import dd2414_brain_v2.msg as brain
import dd2414_text_speech.msg as tts 

from actionlib import SimpleActionClient
from googletrans import Translator
from std_msgs.msg import String
from dd2414_status_update import StatusUpdate

class TranslateConversation:
    def __init__(self):
        self.result   = brain.BrainResult()
        #self.tts_goal = tts.TextToSpeechMultilanguageGoal()

        # Action client of TTS Multilanguages
        #self.ac_ttsm = SimpleActionClient('text_multilanguage_speech', tts.TextToSpeechMultilanguageAction)
        #self.ac_ttsm.wait_for_server()
        self.ac_ollama_response = rospy.Publisher(f"translate/ollama_response",brain.BrainGoal,queue_size=1)

        rospy.loginfo("[Translate Conversation]:Initialized")
        self.string_header = "[Translate Conversation]:"

        # Subscribers
        #rospy.Subscriber("/humans/voices/anonymous_speaker/speech",LiveSpeech,self.asr_result)
        rospy.Subscriber("/tts/ARI_speeking",String,self.ari_speeking_state)
        rospy.Subscriber("/stt/transcript",String,self.stt)

        self.translate_pub = rospy.Publisher("/translate_conversation",String,queue_size=10)

        self.text         = None
        self.data_dic     = None
        self.stop         = False
        self.running      = False
        self.stt_language = "" 
        self.ari_speeking = ""
        self.translating  = ""
        self.data_dic     = ""
        self.translator   = Translator()
        self.languages    = {"spanish":"es","english":"en","french":"fr","german":"de"}

    def ari_speeking_state(self,msg):
        self.ari_speeking = msg.data

    def stt(self,msg):
        if not msg.data:
            return

        try:
            rospy.loginfo(msg.data)
            self.data_dic = msg.data
            data_dic      = json.loads(self.data_dic)
            phrase        = data_dic["translation"]
            stt_language  = data_dic["language"]

            if "stop" in (phrase).lower() and len(phrase.split())<2:
                self.stop = True
                rospy.loginfo("[Translate Conversation]:Stoping Translate Conversation behavior")

                self.result.result = "Success"
                self.translate_pub.publish("")

                return self.result

        except Exception as e:
            rospy.logerr(f"Invalid data:{e}")
            return

    def action(self,goal):
        try:

            if not self.running:
                self.running = True
                self.stop    = False

            #dictonary        = json.loads(goal.in_dic)
            source_language  = str(goal.goal) #dictonary["source"]
            target_language  = str(goal.in_dic)#dictonary["target"]
            
            if self.data_dic != None:
                data_dic     = json.loads(self.data_dic)
                phrase       = data_dic["translation"]
                stt_language = self.languages[(data_dic["language"]).lower()]

                return self.generate_translation(phrase,source_language,target_language,stt_language)
            else:
                self.result.result = "Working"
                self.translate_pub.publish("translating")
        
        except Exception as e:
            rospy.logerr(f"Empty Goal: {e}")
            self.result.result = "Failure"
            self.translate_pub.publish("")

            return self.result

    def preempted(self):
        pass

    def generate_translation(self,phrase,src_language,target_language,language_stt):
        try:
            rospy.loginfo("Translating")
            self.translate_pub.publish("translating")
            self.result.result = "Working"

            if language_stt != None and phrase != None:
                if not self.stop:
                    languages  = [src_language,target_language]

                    if language_stt in languages:
                        to_lang     = languages.index(language_stt-1)
                        result_text = self.translator.translate(phrase, src="en", dest=to_lang)
            
                    else:
                        to_lang     = language_stt
                        result_text = self.translator.translate("Please say it again, I did not understand.", src="en", dest=to_lang)
                    
                    result_text = result_text.text
                    goal        = brain.BrainGoal()
                    goal.goal   = ""
                    goal.in_dic = {"language":to_lang,"intent":"translate","phrase":result_text}

                    # Send audio goal
                    rospy.loginfo("[Ollama Response]:Response sent to TTS")
                    #self.ac_ttsm.send_goal_and_wait(self.tts_goal)
                    rospy.loginfo(goal)
                    self.ac_ollama_response.publish(goal)

                    if self.stop:
                        self.stop    = False
                        self.running = False
                        rospy.loginfo("[Translate Conversation    ]:Returning stop")
                        self.result.result = "Success"
                        
                        return self.result
                
            else:
                rospy.loginfo("[Translate Conversation    ]:Waiting for conversation")

            return self.result

        except Exception as e:
            rospy.logerr(f"No audio detected: {e}")
            self.result.result = "Working"

            return self.result
        

if __name__ == "__main__":
    rospy.init_node("translate_conversation",log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(),TranslateConversation)
    rospy.spin()
