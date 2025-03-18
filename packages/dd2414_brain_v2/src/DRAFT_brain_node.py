#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
import pyhri
class ARI:
    def __init__(self):
        self.current_state = "Idle"
        self.done = False
        self.debug_pub = rospy.Publisher("~dfd_debug",String,queue_size=10)
        self.response_sub = rospy.Subscriber("~response",Bool,self.response_cb)
        self.action_dict= {
                            "greet"                :self.greet,
                            "goodbye"              :self.goodbye,
                            "small talk"           :self.small_talk,
                            "follow user"          :self.follow_user,
                            "provide information"  :self.provide_information,
                            "find object"          :self.find_object,
                            "move to"              :self.move_to,
                            "explore"              :self.explore,
                            "translate"            :self.translate,
                            "other"                :self.other,
                            "stop"                 :self.stop
                            }

    def run_action(self,intent):
        if intent in self.action_dict:
            self.current_state = "Busy"
            result = self.action_dict[intent]()
        

    def stop(self):
        msg = String()
        msg.data = "STOP"
        self.debug_pub.publish(msg)

    def response_cb(self,msg):
        self.done = msg.data

#   def action(self):
#       All actions should keep the execution here until execution is finished 
#        
#
    def greet(self):
        msg = String()
        msg.data = "Greet Action"
        
        self.debug_pub.publish(msg)
        return
    
    def goodbye(self):
        msg = String()
        msg.data = "GoodBye Action"
        self.debug_pub.publish(msg)
        if self.done == True:
            self.current_state = "Success"
        return

    def small_talk(self):
        msg = String()
        msg.data = "Small Talk Action"
        self.debug_pub.publish(msg)
        return
    
    def follow_user(self):
        msg = String()
        msg.data = "Follow User Action"
        self.debug_pub.publish(msg)
        return

    def provide_information(self):
        msg = String()
        msg.data = "Provide Information Action"
        self.debug_pub.publish(msg)
        return
    
    def find_object(self):
        msg = String()
        msg.data = "Find Object Action"
        self.debug_pub.publish(msg)
        return

    def move_to(self):
        msg = String()
        msg.data = "Move To Action"
        self.debug_pub.publish(msg)
        return
    
    def explore(self):
        msg = String()
        msg.data = "Explore Action"
        self.debug_pub.publish(msg)
        return

    def translate(self):
        msg = String()
        msg.data = "Translate Action"
        self.debug_pub.publish(msg)
        return
    
    def other(self):
        msg = String()
        msg.data = "Other Action"
        self.debug_pub.publish(msg)
        return

class Brain:
    def __init__(self):
        rospy.loginfo("Brain Start Initializing")
        self.new_intent = ""
        self.current_intent = self.new_intent
        self.new_dictionary = {}
        self.intent_sub=rospy.Subscriber(rospy.get_param("~IN_intent_topic","~intent"),String, self.intent_cb)
        self.debug_pub=rospy.Publisher("~dfd_debug",String,queue_size=10)
        self.robot = ARI()

        rospy.loginfo("Brain Finished Initializing")

    def intent_cb(self,string_msg):
        self.new_dictionary = json.loads(string_msg.data)
        self.new_intent = self.new_dictionary["intent"]

    def run (self):
        if self.new_intent == "stop":
            #if the intent is Stop we inmediatly call for the stop function
            self.robot.stop()
        else:
            if self.robot.current_state == "Idle" :
                #Parting from an Idle State the robot can start any of the actions we call one action based on that
                self.current_intent = self.new_intent
                self.robot.run_action(self.new_intent)

            elif self.robot.current_state == "Busy" :
                self.robot.run_action(self.current_intent)
            elif self.robot.current_state == "Success":
                rospy.loginfo("Mission Succed")
    
    def shutdown(self):
        rospy.loginfo("Shutting Down Brain Node")
        self.robot.stop()
                    


if __name__ == '__main__':

    rospy.init_node('brain',anonymous=False)
    node = Brain()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        node.run()
        rospy.loginfo("State : " + node.robot.current_state + " | Intent : " + node.current_intent)
        rate.sleep()
    
    node.shutdown()
    