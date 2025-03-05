#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
import time

class ARI:
    def __init__(self):
        self.current_state = "Idle"
        self.last_result = "Done"
        self.current_intent = "stop"
        self.count = 0

        #Goodbye Subscriber for Example will be replaced with a Service call
        self.response_sub = rospy.Subscriber("~response",Bool,self.response_cb)
        self.response_msg = Bool()

        #To Add More Behaviors just add them to this dictionary and then add the corresponding function
        self.action_dict= {
                            "hello"                :self.hello,
                            "goodbye"              :self.goodbye,
                            "stop"                 :self.stop
                            }
        

    def response_cb(self,response_msg):
        self.response_msg = response_msg.data

    def run_action(self,intent):
        if intent == "stop": #Give Priority to the Stop Action
            self.current_intent = "stop"
            self.current_state = "Idle"
            self.last_result = self.action_dict["stop"]()

        elif self.current_state == "Idle" and intent in self.action_dict: #If Robot is Idle and the requested action is valid
            self.current_intent = intent #Save the Current action
            self.current_state = "Busy"
            self.last_result = self.action_dict[self.current_intent]()

        elif self.current_state == "Busy" and self.last_result == "Working":
            self.last_result = self.action_dict[self.current_intent]()

        elif self.current_state == "Busy" and self.last_result == "Done":
            self.current_state = "Success"

        elif self.current_state == "Success" and self.last_result == "Done":
            self.current_state = "Idle"
            self.idle()

        rospy.loginfo("State: "+ self.current_state + " | Current Action: " + self.current_intent + " | Result: " + self.last_result + " | Intent: " + intent)
    
    def stop (self):
        result = "Done"
        #rospy.loginfo("State: "+ self.current_state + " Action: " + "STOP" + " Result: " + result)
        return "Done"
    
    def idle (self):
        result = "Done"
        #rospy.loginfo("State: "+ self.current_state + " Action: " + "IDLE" + " Result: " + result)
        return result
    
    def goodbye(self):
        while self.response_msg != True:
            result = "Working"
            rospy.loginfo("sleep")
            time.sleep(2)
        
        self.response_msg = False
        result = "Done"
        #rospy.loginfo("State: "+ self.current_state + " Action: " + "GOODBYE" + " Result: " + result)
        return result
    
    def hello(self):

        if self.count == 10:
            result="Done"
            self.count = 0
        else:
            result="Working"
        self.count += 1

        #rospy.loginfo("State: "+ self.current_state + " Action: " + "HELLO" + " Result: " + result)
        return result

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
        self.robot.run_action(self.intent)
        self.intent = ""

    def shutdown (self):
        rospy.loginfo("Shutting Down Brain Node")
        self.robot.run_action("stop")

if __name__ == '__main__':
    rospy.init_node('brain',anonymous=False)
    node = Brain()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        node.run()
        rate.sleep()

    node.shutdown()

        
