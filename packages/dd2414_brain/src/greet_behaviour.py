#!/usr/bin/env python3

import actionlib
import py_trees
import py_trees_ros
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import dd2414_brain.msg as brain

class GreetBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):

        super(GreetBehaviour, self).__init__(name=name)
        
        self.blackboard = py_trees.blackboard.Blackboard()
        self.person_looking_at_ari = ""

        self.person_found_sub   = rospy.Subscriber('person_looking_at_robot', String, self.person_found_cb, queue_size=10)
        self._as_save_name = actionlib.SimpleActionClient("/face_recognition", brain.BrainAction)

        self.translating = False

    def setup(self, timeout):
        rospy.logdebug("[TestBehaviour] {}.setup()".format(self.name))
        self.feedback_message = "behaviour created"

        self.counter = 0
        return True

    def update(self) -> py_trees.common.Status:

        rospy.logdebug(self.string_header + "Greet")
        #Turns to Look at the speaker

        
        
        #Waits for the server to be available
        if self._as_save_name.wait_for_server(rospy.Duration(self.timeout)):
                ActionGoal = brain.BrainGoal()
                ActionGoal.goal = "unknown"

                #If it did actually manage to see someone then it tries to recognize the user
                if self.person_looking_at_ari:
                    self._as_save_name.send_goal(ActionGoal,done_cb=self.cb_done,active_cb=self.cb_active,feedback_cb=self.cb_feedback)
                else: #If the turning fails because user moved or it turned to the wrong direction
                    self.last_result = "Failure"
                    self.last_data = {"name" : "unknown"}
        else: #If the service was not available or it timed out
            rospy.logdebug(self.string_header + "Greet TIMEOUT")
            self.last_result = "Failure"
        

        # When behaviour is done
        if(self.counter > 10):
            self.blackboard.set(self.name, False)
            self.feedback_message = "success"
            return py_trees.common.Status.SUCCESS

        rospy.logdebug("%s.update()" % self.__class__.__name__)
        self.feedback_message = "running"
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):

        rospy.logdebug(
            "{}.terminate({})".format(
                self.name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )

        #self.feedback_message = "cleared"

    def person_found_cb(self,msg):
        self.person_looking_at_ari = msg.data != ""

if __name__ == '__main__':
    
    rospy.init_node('test_behaviour_node',anonymous=False)
    rospy.spin()
