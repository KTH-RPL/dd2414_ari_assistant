#!/usr/bin/env python3

import json
import actionlib
import py_trees
import py_trees_ros
import rospy
import dd2414_brain_v2.msg as brain
from std_msgs.msg import String
from std_msgs.msg import Bool

class StopBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, action_dict: dict):

        super(StopBehaviour, self).__init__(name=name)

        self.blackboard = py_trees.blackboard.Blackboard()
        self.counter = 0
        self.action_dict = action_dict

    def setup(self, timeout):
        rospy.logdebug("[StopBehaviour] {}.setup()".format(self.name))
        self.feedback_message = "behaviour created"
        return True

    def update(self) -> py_trees.common.Status:

        for action in self.action_dict:
            if self.blackboard.get(action) == True and action != "stop:":
                self.blackboard.set(action, False)

        rospy.sleep(2)

        return py_trees.common.Status.SUCCESS


    def terminate(self, new_status: py_trees.common.Status):

        rospy.logdebug(
            "{}.terminate({})".format(
                self.name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )

    def input_cb(self, data):
        rospy.loginfo(f"[STOP    ]: Behaviour received input {data}")
        self.goal = data

if __name__ == '__main__':
    
    rospy.init_node('stop_behaviour_node',anonymous=False)
    rospy.spin()
