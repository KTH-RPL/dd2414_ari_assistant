#!/usr/bin/env python3

import rospy
import actionlib
import dd2414_brain_v2.msg as brain
import py_trees

class StatusUpdate(py_trees.behaviour.Behaviour):
    

    def __init__(self,name,NodeClass):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,brain.BrainAction,execute_cb=self.execute_cb,auto_start=False)
        self._as.start()
        self._feedback = brain.BrainFeedback()
        self._result = brain.BrainResult()
        # INSTANTIATE NODE HERE
        self.node = NodeClass()

        self.input_sub = rospy.Subscriber(f"/brain{self._action_name}", brain.BrainGoal, self.input_cb)
        rospy.logdebug(f"Subscribed to /brain{self._action_name}")
        self.goal = None
        self.previous_goal = None

        self.ollama_response_client = actionlib.SimpleActionClient("/ollama_response",brain.BrainAction)

        self.rate = rospy.Rate(2)

        if hasattr(self.node,"string_header"):
            self.string_header = self.node.string_header
        else:
            self.string_header = ""

        rospy.loginfo(self.string_header + "Action Server " + self._action_name + " initialized.")
    
    def execute_cb(self,goal):

        #Insert Code for new GOAL SETUP HERE

        ###################################
        rospy.loginfo(self.string_header + "Starting Execution of " + self._action_name)
        
        
        self._result.result = "Working" #Variable to store the status

        if(goal.goal != ''):
            self.goal = goal

        if(self._action_name != "/ollama_response"):
            self.ollama_response_client.send_goal(self.goal)

        self.previous_goal = self.goal

        while(self._result.result == "Working" and not rospy.is_shutdown()):

            #If a new goal has been sent, preempt to cancel current goal
            if(self.previous_goal != self.goal):
                self.node.preempted()
                self.previous_goal = self.goal

            if self._as.is_preempt_requested():

                #If goal has been canceled perform necessary shutdown behavior
                self.node.preempted()
                ##################################################
                self._result.result = "Failure"
                self._as.set_preempted(self._result)
                rospy.loginfo(self.string_header + "Goal Preempted.")
                return
            else:
 
                self._feedback.feedback = self._result.result
                self._feedback.in_dic = self._result.in_dic
                self._as.publish_feedback(self._feedback)
                self._result = self.node.action(self.goal)
                self.rate.sleep()
                    
            
            self.rate.sleep()

        if self._result.result == "Success":
                
            rospy.loginfo(self.string_header + "Action Server " + self._action_name + " Succeded.")
            self._as.set_succeeded(self._result)
        else:
            self._result.result = "Failure"
            rospy.loginfo("Action Server " + self._action_name + " Aborted.")
            self._as.set_aborted(self._result)

        self.rate.sleep()

    def input_cb(self, data):
        rospy.loginfo(f"[{self._action_name}]: StatusUpdate received input {data}")
        self.goal = data
