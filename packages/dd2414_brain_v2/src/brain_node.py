#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
import time

import actionlib
import dd2414_brain_v2.msg as brain

class ARI:
    def __init__(self):
        self.current_state = "Idle"
        self.last_result = ""
        self.current_intent = ""
        self.emergency_stop = False

        self.person_looking_at_ari = ""
        self.timeout = rospy.get_param("~timeout",10)
        self.look_at_person_enable = True

        self.person_found_sub   = rospy.Subscriber('person_looking_at_robot', String, self.person_found_cb, queue_size=10)
        self.brain_state_pub    = rospy.Publisher('/brain/state',String,queue_size=10)
        self.brain_user_name_pub= rospy.Publisher('/brain/user_name',String,queue_size=10)
        
        self._as_go_to_location = actionlib.SimpleActionClient("/nav_move_base_server",brain.BrainAction)
        self._as_text_speech    = actionlib.SimpleActionClient("/text_speech",brain.BrainAction)
        self._as_save_name      = actionlib.SimpleActionClient("/face_recognition_node", brain.BrainAction)
        self._as_find_speaker   = actionlib.SimpleActionClient("/ari_turn_to_speaker",brain.BrainAction)
        self._as_follow_user    = actionlib.SimpleActionClient("/follow_user",brain.BrainAction)
        self._as_move_to_person = actionlib.SimpleActionClient("/body_orientation_listener",brain.BrainAction)
        self._as_look_at_pèrson = actionlib.SimpleActionClient("/face_gaze_tracker",brain.BrainAction)


        #To Add More Behaviors just add them to this dictionary and then add the corresponding function
        self.action_dict= {
                            "stop"                 :self.stop,
                            "remember user"        :self.name_assign,
                            "go to"                :self.go_to_location,
                            "find speaker"         :self.find_speaker,
                            "follow user"          :self.follow_user,
                            "translate"            :self.translate,
                            "provide information"  :self.provide_information,
                            "speech"               :self.text_to_speech,
                            "greet"                :self.greet,
                            "goodbye"              :self.greet
                            }
        
    def response_cb(self,response_msg):
        self.response_msg = response_msg.data

    def run_action(self,intent,input):
        result = ""
        rospy.loginfo("BEFORE State: "+ self.current_state + " | Current Action: " + self.current_intent + " | Result: " + self.last_result + " | Intent: " + intent)
        if intent == "stop" or self.emergency_stop: #Give Priority to the Stop Action
            self.current_intent = "stop"
            self.current_state = "Busy"
            self.action_dict["stop"]({})

        elif self.current_state == "Idle" and intent in self.action_dict: #If Robot is Idle and the requested action is valid
            self.current_intent = intent #Save the Current action
            self.current_state = "Busy"

            if intent in ("follow user","go to","provide information","translate","find speaker") and self.look_at_person_enable:
                self.look_at_person({"input":"stop"})
            elif not self.look_at_person_enable:
                self.look_at_person({"input":"start"})
            self.action_dict[self.current_intent](input)

        elif self.current_state == "Busy" and self.last_result == "Working": #If robot is Working on the task at hand call the function to ask for an update
            pass

        elif self.current_state == "Busy" and self.last_result == "Success": #If the robot just recieved a Success result from the action it was performing; we could skip this state transition to have it directly go into idle
            self.current_state = "Done"
            result = self.last_result
            if self.current_intent == "greet" and "name" in self.last_data:
                rospy.loginfo("Name sent to LLM: " + json.dumps(self.last_data))
                self.brain_user_name_pub.publish(self.last_data["name"])


        elif self.current_state == "Busy" and self.last_result == "Failure": #If the robot just recieved a Success result from the action it was performing; we could skip this state transition to have it directly go into idle
            self.current_state = "Done"
            result = self.last_result
            

        elif self.current_state == "Done" : #Its just the next state after success
            self.current_state = "Idle"
            self.current_intent = ""
            result = self.last_result
            self.last_result = ""


        self.brain_state_pub.publish(self.current_state)
        rospy.loginfo("AFTER  State: "+ self.current_state + " | Current Action: " + self.current_intent + " | Result: " + self.last_result + " | Intent: " + intent)
        return result
    
    def stop (self,input):
        self.last_result = "Success"
        self.last_data = {}
    
    def idle (self,input):
        self.last_result = "Success"
        self.last_data = {}
    
    def cb_done(self,state,result):
        self.last_result=result.result
        #rospy.loginfo(self.current_intent)
        #rospy.loginfo(result)
        if result.in_dic != '':
            self.last_data = json.loads(result.in_dic)
        else:
            self.last_data = {}

    def cb_feedback(self,feedback):
        self.last_result=feedback.feedback
        #rospy.loginfo(feedback)
        if feedback.in_dic != '':
            self.last_data = json.loads(feedback.in_dic)
        else:
            self.last_data = {}

    def cb_active(self):
        self.last_result = "Working"
        

    def name_assign(self,input):
        if self._as_save_name.wait_for_server(rospy.Duration(self.timeout)):
            #rospy.loginfo(input)
            ActionGoal = brain.BrainGoal()
            ActionGoal.goal = input["input"]
            self._as_save_name.send_goal(ActionGoal, done_cb = self.cb_done, active_cb = self.cb_active, feedback_cb = self.cb_feedback)
        else:
            self.last_result = "Failure"
            self.last_data = {}
        
    
    def person_found_cb(self,msg):
        self.person_looking_at_ari = msg.data != ""

    def text_to_speech(self,input):
        if self._as_text_speech.wait_for_server(rospy.Duration(self.timeout)):
        #Change this for the string input.goal
            rospy.loginfo("TEXT TO SPEECH")
            ActionGoal = brain.BrainGoal()
            ActionGoal.goal = input["input"]
            self._as_text_speech.send_goal(ActionGoal,done_cb=self.cb_done,active_cb=self.cb_active,feedback_cb=self.cb_feedback)
        else:
            self.last_result = "Failure"

    def go_to_location(self,input):
        if self._as_go_to_location.wait_for_server(rospy.Duration(self.timeout)):
        #Change this for the string input.goal
            rospy.loginfo(input)
            ActionGoal = brain.BrainGoal()
            ActionGoal.goal = input["input"]
            self._as_go_to_location.send_goal(ActionGoal,done_cb=self.cb_done,active_cb=self.cb_active,feedback_cb=self.cb_feedback)
        #wait = self._as_go_to_location.wait_for_result()
        #result = self._as_go_to_location.get_result()
        else:
            self.last_result = "Failure"

    def find_speaker(self, input):
        if self._as_find_speaker.wait_for_server(rospy.Duration(self.timeout)):
            rospy.loginfo("FIND SPEAKER")
            ActionGoal = brain.BrainGoal()
            self._as_find_speaker.send_goal(ActionGoal,done_cb=self.cb_done,active_cb=self.cb_active,feedback_cb=self.cb_feedback)
        else:
            self.last_result = "Failure"

    def follow_user(self, input):
        if not self.person_looking_at_ari:
            self.action_dict["find speaker"]({})
        if self._as_follow_user.wait_for_server(rospy.Duration(self.timeout)):
            ActionGoal = brain.BrainGoal()
            self._as_follow_user.send_goal(ActionGoal,done_cb=self.cb_done,active_cb=self.cb_active,feedback_cb=self.cb_feedback)
        else:
            self.last_result = "Failure"

    def move_to_person(self,input):
        if self._as_move_to_person.wait_for_server(rospy.Duration(self.timeout)):
            ActionGoal = brain.BrainGoal()
            self._as_move_to_person.send_goal(ActionGoal,done_cb=self.cb_done,active_cb=self.cb_active,feedback_cb=self.cb_feedback)
        else:
            self.last_result = "Failure" 

    def translate(self, input):
        if not self.person_looking_at_ari:
            self.action_dict["find speaker"]({})
        if self.person_looking_at_ari :
            self.move_to_person({})
        else:
            self.last_result = "Failure"
        #if move_to_person == "Success" 

    def provide_information(self, input):
        if not self.person_looking_at_ari:
            self.action_dict["find speaker"]({})
        if self.person_looking_at_ari :
            self.move_to_person({})
        else:
            self.last_result = "Failure"

        #if move_to_person == "Success" 

    def greet(self, input):
        #Turns to Look at the speaker
        if not self.person_looking_at_ari:
            self.action_dict["find speaker"]({})
        
        #Waits for the server to be available
        if self._as_save_name.wait_for_server(rospy.Duration(self.timeout)):
                #rospy.loginfo(input)
                ActionGoal = brain.BrainGoal()
                ActionGoal.goal = "unknown"

                #If it did actually manage to see someone then it tries to recognize the user
                if self.person_looking_at_ari:
                    self._as_save_name.send_goal(ActionGoal,done_cb=self.cb_done,active_cb=self.cb_active,feedback_cb=self.cb_feedback)
                else: #If the turning fails because user moved or it turned to the wrong direction
                    self.last_result = "Failure"
                    self.last_data = {"name" : "unknown"}
        else: #If the service was not available or it timed out
            self.last_result = "Failure"
    
    def look_at_person(self,input):
        if self._as_look_at_pèrson.wait_for_server(rospy.Duration(self.timeout)):
            ActionGoal = brain.BrainGoal()
            ActionGoal.goal = input["input"]
            self._as_look_at_pèrson.send_goal(ActionGoal)
            if ActionGoal.goal == "start":
                self.look_at_person_enable = True 
            elif ActionGoal.goal == "stop":
                self.look_at_person_enable = False





#Dont MOVE ANYTING FROM HERE
class Brain:
    def __init__(self):
        rospy.loginfo("Brain Start Initializing")

        self.intent_sub=rospy.Subscriber(rospy.get_param("~IN_intent_topic","~intent"),String, self.intent_cb)
        self.intent_dict = {}
        self.intent = ""

        self.robot = ARI()

        rospy.loginfo("Brain Finished Initializing")

    def intent_cb(self,string_msg):
        self.intent_dict = json.loads(string_msg.data)
        self.intent = self.intent_dict["intent"]

    def run(self):
        result = self.robot.run_action(self.intent,self.intent_dict)
        #if result == "Success" or result == "Failure":
        self.intent = ""
        

    def shutdown (self):
        rospy.loginfo("Shutting Down Brain Node")
        self.robot.run_action("stop",{})

if __name__ == '__main__':
    rospy.init_node('brain',anonymous=False)
    node = Brain()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        node.run()
        rate.sleep()

    node.shutdown()

        
