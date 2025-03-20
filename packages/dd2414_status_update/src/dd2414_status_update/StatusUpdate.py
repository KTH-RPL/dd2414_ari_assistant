#!/usr/bin/env python3

import rospy
import actionlib
import dd2414_brain_v2.msg as brain

class StatusUpdate:
    _feedback = brain.BrainFeedback()
    _result = brain.BrainResult()

    def __init__(self,name,NodeClass):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,brain.BrainAction,execute_cb=self.execute_cb,auto_start=False)
        self._as.start()

        # INSTANTIATE NODE HERE
        self.node = NodeClass()

        #####################################
        rospy.loginfo("Action Server " + self._action_name + " initialized.")
    
    def execute_cb(self,goal):
        #Insert Code for new GOAL SETUP HERE

        ###################################
        rospy.loginfo("Starting Execution of " + self._action_name)
        status = "Working" #Variable to store the status

        if self._as.is_preempt_requested():
            #If goal has been canceled perform necessary shutdown behavior
            self.node.preempted()
            ##################################################
            self._as.set_preempted()
            rospy.loginfo("Goal Preempted.")
        else:
            while status == "Working":
                if self._as.is_preempt_requested():
                    #If goal has been canceled perform necessary shutdown behavior
                    self.node.preempted()
                    ##################################################
                    self._as.set_preempted()
                    rospy.loginfo("Goal Preempted.")

                self._feedback.feedback = status
                self._as.publish_feedback(self._feedback)
                status = self.node.action(goal)
            if status == "Success":
                self._result.result = status
                rospy.loginfo("Action Server " + self._action_name + " Succeded.")
                self._as.set_succeeded(self._result)
            else:
                self._result.result = "Failure"
                rospy.loginfo("Action Server " + self._action_name + " Aborted.")
                self._as.set_aborted(self._result)