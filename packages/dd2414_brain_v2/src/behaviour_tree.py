#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
import time
from test_behaviour import TestBehaviour

import actionlib
import dd2414_brain_v2.msg as brain

import py_trees



#Dont MOVE ANYTING FROM HERE
class Brain:
    def __init__(self):
        rospy.loginfo("Brain Start Initializing")

        self.intent_sub=rospy.Subscriber(rospy.get_param("~IN_intent_topic","~intent"),String, self.intent_cb)
        self.intent_dict = {}
        self.intent = ""

        self.current_requested_intent = None

        # self.robot = ARI()

        rospy.loginfo("Brain Finished Initializing")

        # Set up behaviours and construct tree
        root = py_trees.composites.Sequence("Selector", memory=False)
        behaviour_a = TestBehaviour(name="Test behaviour")
        behaviour_b = py_trees.behaviours.Success(name="Success behaviour")

        root.add_children([behaviour_a, behaviour_b])

        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)
        print(py_trees.display.print_ascii_tree(root=self.behaviour_tree.root))

        self.behaviour_tree.setup(timeout=15)

    def print_tree(self, tree: py_trees.trees.BehaviourTree) -> None:
        # Print the behaviour tree and its current status.
        print(py_trees.display.print_ascii_tree(root=tree.root, show_status=True))

    def intent_cb(self,string_msg):
        self.intent_dict = json.loads(string_msg.data)
        self.intent = self.intent_dict["intent"]

        self.current_requested_intent = self.intent
        print('Requested intent ', self.intent)

    """def run(self):
        result = self.robot.run_action(self.intent,self.intent_dict)
#        if result == "Success" or result == "Failure":
        self.intent = "" """
    
    def action_requested(self, action):
        return self.current_requested_intent == action

    def shutdown (self):
        rospy.loginfo("Shutting Down Brain Node")
        #self.robot.run_action("stop",{})

if __name__ == '__main__':
    rospy.init_node('brain',anonymous=False)
    node = Brain()
    rate = rospy.Rate(10)

    while(not rospy.is_shutdown()):
        try:
            node.behaviour_tree.tick(
                pre_tick_handler=None,
                post_tick_handler=node.print_tree
            )

        except KeyboardInterrupt:
            node.behaviour_tree.interrupt()

        rate.sleep()

    node.shutdown()