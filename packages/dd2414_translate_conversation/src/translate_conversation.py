#!/usr/bin/python3
import rospy
import json
import dd2414_brain.msg as brain
import dd2414_text_speech.msg as tts 

from actionlib import SimpleActionClient
import actionlib
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
        self.ac_ollama_response = actionlib.SimpleActionClient("/ollama_response",brain.BrainAction)
        rospy.loginfo("[Translate Conversation]:Initialized")
        self.string_header = "[Translate Conversation]:"

        # Subscribers
        #rospy.Subscriber("/humans/voices/anonymous_speaker/speech",LiveSpeech,self.asr_result)
        rospy.Subscriber("/tts/ARI_speeking",String,self.ari_speeking_state)
        rospy.Subscriber("/stt/transcript",String,self.stt,queue_size=1)

        self.translate_pub = rospy.Publisher("/translate_conversation",String,queue_size=10)

        self.text         = None
        self.data_dic     = None
        self.stop         = False
        self.running      = False
        self.process      = True
        self.stt_language = "" 
        self.ari_speeking = ""
        self.translating  = ""
        self.data_dic     = ""
        self.translator   = Translator()
        self.languages    = {"spanish":"es","english":"en","french":"fr","german":"de","deutsch":"de","japanese":"jp","swedish":"sv"}

    def ari_speeking_state(self,msg):
        self.ari_speeking = msg.data

    def stt(self,msg):
        if not msg.data:
            return

        try:
            if self.ari_speeking != "speaking" and self.process == True and self.running:
                #rospy.loginfo(msg.data)
                self.data_dic = msg.data
                data_dic      = json.loads(self.data_dic)
                phrase        = data_dic["translation"]
                stt_language  = data_dic["language"]

                if "stop" in (phrase).lower() and len(phrase.split())<2:
                    self.stop = True
                    self.running = False
                    rospy.loginfo("[Translate Conversation]:Stoping Translate Conversation behavior")

                    self.result.result = "Success"
                    self.translate_pub.publish("")

                    return self.result
            else:
                self.data_dic = None

        except Exception as e:
            rospy.logerr(f"Invalid data:{e}")
            return

    def action(self,goal):
        try:

            if not self.running:
                self.running = True
                self.stop    = False

            #dictonary        = json.loads(goal.in_dic)
            target_language  = str(goal.goal).replace("\"","")#dictonary["source"]
            language_dic     = json.loads(goal.in_dic)
            #source_language  = str(language_dic["language"]).replace("\"","")#dictonary["target"]
            #rospy.loginfo(source_language)
            source_language  = target_language.split(",")
            target_language  = source_language[0]
            source_language  = source_language[1]
#            rospy.loginfo("######################")
#            rospy.loginfo(self.data_dic) 
#            rospy.loginfo("######################")
            if self.data_dic != None:
                data_dic     = json.loads(self.data_dic)
                phrase       = data_dic["translation"]
                target_language = self.languages[(target_language).lower()]
                stt_language = data_dic["language"]

                self.process = False
                return self.generate_translation(phrase,source_language,target_language,stt_language)
            else:
                self.result.result = "Working"
                self.translate_pub.publish("translating")
                return self.result
        
        except Exception as e:
            rospy.logerr(f"Empty Goal: {e}")
            self.result.result = "Failure"
            self.translate_pub.publish("")

            return self.result

    def preempted(self):
        self.translate_pub.publish("")
        pass

    def generate_translation(self,phrase,src_language,target_language,language_stt):
        try:
            rospy.loginfo("Translating")
            self.translate_pub.publish("translating")
            self.result.result = "Working"

            if language_stt != None and phrase != None and self.data_dic != None:
                if not self.stop:
                    self.data_dic = None
                    languages  = [src_language,target_language]
                    rospy.loginfo(languages)
                    rospy.loginfo(language_stt)
                    rospy.loginfo("Before Translate object")
                    rospy.loginfo(f"Phrase: {phrase}")
                    if language_stt in languages:
                        index = languages.index(language_stt)-1
                        to_lang     = languages[index]
                        result_text = self.translator.translate(phrase, src="en", dest=to_lang)
                        result_text = result_text.text
                    else:
                        to_lang     = "en"
                        result_text = "Please say it again, I did not understand." #self.translator.translate("Please say it again, I did not understand.", src="en", dest=to_lang)
                    
                    rospy.loginfo("After Translate Object")
                    rospy.loginfo(result_text)
                    goal        = brain.BrainGoal()
                    goal.goal   = "a"
                    goal.in_dic = {"language":to_lang,"intent":"translate","phrase":result_text}
                    rospy.loginfo("[Ollama Response]:Response sent to TTS")
                    rospy.loginfo(goal)
                    goal.in_dic = json.dumps(goal.in_dic)
                    # Send audio goal
                    #self.ac_ttsm.send_goal_and_wait(self.tts_goal)
                    self.ac_ollama_response.send_goal_and_wait(goal)
                    state = self.ac_ollama_response.get_state()

                    if self.stop:
                        self.stop    = False
                        self.running = False
                        rospy.loginfo("[Translate Conversation    ]:Returning stop")
                        self.result.result = "Success"
                        
                        return self.result
                
            else:
                rospy.loginfo("[Translate Conversation    ]:Waiting for conversation")

            self.process = True
            return self.result

        except Exception as e:
            rospy.logerr(f"No audio detected: {e}")
            self.result.result = "Working"

            return self.result
        

if __name__ == "__main__":
    rospy.init_node("translate_conversation",log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(),TranslateConversation)
    rospy.spin()
