#!/usr/bin/env python3

import py_trees
import py_trees_ros
import rospy
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
            self.blackboard.set(action, False)

        print("BLACKBOARD: ", self.blackboard)

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

if __name__ == '__main__':
    
    rospy.init_node('stop_behaviour_node',anonymous=False)
    rospy.spin()
