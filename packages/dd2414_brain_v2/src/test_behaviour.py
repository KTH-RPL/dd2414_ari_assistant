#!/usr/bin/env python3

import py_trees
import py_trees_ros
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

class TestBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):

        super(TestBehaviour, self).__init__(name=name)
        
        self.blackboard = py_trees.blackboard.Blackboard()
        self.counter = 0

    def setup(self, timeout):
        rospy.logdebug("[TestBehaviour] {}.setup()".format(self.name))
        self.feedback_message = "behaviour created"
        return True

    def update(self) -> py_trees.common.Status:
        print(self.name)
        self.counter+=1

        # INPUTS print('---------', self.blackboard.inputs[self.name])

        # When behaviour is done
        if(self.counter > 10):
            self.blackboard.set(self.name, False)
            self.feedback_message = "success"
            counter = 0
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

if __name__ == '__main__':
    
    rospy.init_node('test_behaviour_node',anonymous=False)
    rospy.spin()
