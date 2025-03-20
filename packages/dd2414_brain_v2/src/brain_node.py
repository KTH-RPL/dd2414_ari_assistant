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
        self.last_result = "Success"
        self.current_intent = "stop"

        self.timeout = rospy.get_param("~timeout",10)

        self.person_found_sub = rospy.Subscriber('person_looking_at_robot', String, self.person_found_cb, queue_size=10)
        self._as_go_to_location = actionlib.SimpleActionClient("/nav_move_base_server",brain.BrainAction)
        self._as_find_speaker = actionlib.SimpleActionClient("/ari_turn_to_speaker",brain.BrainAction)
        self._as_follow_user = actionlib.SimpleActionClient("/follow_user",brain.BrainAction)

        #To Add More Behaviors just add them to this dictionary and then add the corresponding function
        self.action_dict= {
                            "stop"                 :self.stop,
                            "name"                 :self.name_assign,
                            "go to"                :self.go_to_location,
                            "find speaker"         :self.find_speaker,
                            "follow user"          :self.follow_user
                            }
        
    def response_cb(self,response_msg):
        self.response_msg = response_msg.data

    def run_action(self,intent,input):
        if intent == "stop": #Give Priority to the Stop Action
            self.current_intent = "stop"
            self.current_state = "Idle"
            self.action_dict["stop"]({})

        elif self.current_state == "Idle" and intent in self.action_dict: #If Robot is Idle and the requested action is valid
            self.current_intent = intent #Save the Current action
            self.current_state = "Busy"
            self.action_dict[self.current_intent](input)

        elif self.current_state == "Busy" and self.last_result == "Working": #If robot is Working on the task at hand call the function to ask for an update
            #self.action_dict[self.current_intent](input)
            pass

        elif self.current_state == "Busy" and self.last_result == "Success": #If the robot just recieved a Success result from the action it was performing; we could skip this state transition to have it directly go into idle
            self.current_state = "Success"

        elif self.current_state == "Busy" and self.last_result == "Failure": #If the robot just recieved a Success result from the action it was performing; we could skip this state transition to have it directly go into idle
            self.current_state = "Idle"
            self.current_intent = "stop"
            self.action_dict["stop"]({})

        elif self.current_state == "Success" and self.last_result == "Success": #Its just the next state after success
            self.current_state = "Idle"
            self.idle({}) #Take any actions needed for it to be idling

        rospy.loginfo("State: "+ self.current_state + " | Current Action: " + self.current_intent + " | Result: " + self.last_result + " | Intent: " + intent)
    
    def stop (self,input):
        result = "Success"
        #rospy.loginfo("State: "+ self.current_state + " Action: " + "STOP" + " Result: " + result)
        return "Success"
    
    def idle (self,input):
        result = "Success"
        #rospy.loginfo("State: "+ self.current_state + " Action: " + "IDLE" + " Result: " + result)
        return result
    
    def cb_done(self,state,result):
        self.last_result=result.result

    def cb_feedback(self,feedback):
        self.last_result=feedback.feedback

    def cb_active(self):
        self.last_result = "Working"

    def name_assign(self,input):
        pass
    
    def person_found_cb(self,msg):
        self.person_looking_at_ari = msg.data != ""

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
        else:
            if self._as_follow_user.wait_for_server(rospy.Duration(self.timeout)):
                self._as_follow_user.wait_for_server()
                ActionGoal = brain.BrainGoal()
                self._as_follow_user.send_goal(ActionGoal,done_cb=self.cb_done,active_cb=self.cb_active,feedback_cb=self.cb_feedback)
            else:
                self.last_result = "Failure"


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
        self.robot.run_action(self.intent,self.intent_dict)
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

        
