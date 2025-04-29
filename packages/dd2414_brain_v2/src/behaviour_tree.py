#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
import time
from test_behaviour import TestBehaviour
from greet_behaviour import GreetBehaviour
import dd2414_brain_v2.msg as brain
from dd2414_brain_v2.msg import BrainAction 

import actionlib


import py_trees
import py_trees_ros


#Dont MOVE ANYTING FROM HERE
class Brain:
    def __init__(self):

        rospy.loginfo("Brain Start Initializing")

        self.intent_sub=rospy.Subscriber(rospy.get_param("~IN_intent_topic","~intent"),String, self.intent_cb)

        self.intent_dict = {}
        self.intent = ""

        self.blackboard = py_trees.blackboard.Blackboard()
        self.current_requested_intent = None

        self.namespace_dict = {
            "test"                 :"/test",
            #"stop"                 :self.stop,
            #"remember user"        :self.name_assign,
            "go to"                :"/nav_move_base_server",
            "find speaker"         :"/ari_turn_to_speaker",
            "follow user"          :"/follow_user",
            #"translate"            :self.translate,
            #"provide information"  :self.provide_information,
            #"speech"               :self.text_to_speech,
            #"greet"                :greet,
            #"goodbye"              :self.greet
        }

        self.behaviours = {}
        self.publishers_dict = {}

        for action in self.namespace_dict:
            behaviour = py_trees_ros.actions.ActionClient(
                name=action,
                action_namespace=self.namespace_dict[action],
                action_spec=brain.BrainAction,
                action_goal=brain.BrainGoal())
            self.behaviours[action] = behaviour

            publisher = rospy.Publisher(f"/brain{self.namespace_dict[action]}", brain.BrainGoal, queue_size=1)
            self.publishers_dict[action] = publisher

        follow_user_behaviour = py_trees.Sequence(
            "Find speaker, then follow user", 
            [self.behaviours["find speaker"], self.behaviours["follow user"]])        

        # Actions in order of priority (higher priority are further up)
        self.action_dict = {
            "test"                 :TestBehaviour(name="test behaviour"),
            #"stop"                 :self.stop,
            #"remember user"        :self.name_assign,
            "go to"                :self.behaviours['go to'],
            "find speaker"         :self.behaviours['find speaker'],
            "follow user"          :follow_user_behaviour,
            #"translate"            :self.translate,
            #"provide information"  :self.provide_information,
            #"speech"               :self.text_to_speech,
            #"greet"                :greet,
            #"goodbye"              :self.greet
            }
            

        greetBehaviourTree = py_trees.Selector(
            name="Greet behaviour",
            children=[
                py_trees.Sequence("Ari Looking at person?",
                [
                    #LookAtSpeakerBehaviour,
                    TestBehaviour(name="test behaviour") #Look at speaker behaviour
                ]),
                GreetBehaviour("Greet behaviour")
            ])
        
        greet_goal = brain.BrainGoal()
        greet_goal.goal = "unknown"        

        for action in self.action_dict:
            self.blackboard.set(action, False)

        # Set up behaviours and construct tree
        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        # For every action, set up condition (action requested) and behaviour (action)
        for action in self.action_dict:
            # Create condition to check if action is requested
            condition = py_trees.blackboard.CheckBlackboardVariable(
                name=f"{action} action not requested?",
                variable_name=action,
                expected_value=False,
            )
            # Create behaviour to be executed if action is requested
            behaviour = self.action_dict[action]

            set_action_requested_to_false = py_trees.blackboard.SetBlackboardVariable(
                name="Set Action requested back to false",
                variable_name=action,
                variable_value=False
            )

            # Add condition and behaviour to tree
            root.add_children([
                py_trees.composites.Selector(
                    f"Execute {action} action if requested", 
                    [
                        condition, 
                        py_trees.composites.Sequence(
                            f"Execute {action} action and set action requested to false", 
                            [
                            behaviour, 
                            set_action_requested_to_false
                            ]),
                    ]),
                ])

        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)
        print(py_trees.display.print_ascii_tree(root=self.behaviour_tree.root))

        self.behaviour_tree.setup(timeout=15)
        rospy.loginfo("Brain Finished Initializing")

    def print_tree(self, tree: py_trees.trees.BehaviourTree) -> None:
        # Print the behaviour tree and its current status.
        print(py_trees.display.print_ascii_tree(root=tree.root, show_status=True))

    def intent_cb(self,string_msg):

        self.intent_dict = json.loads(string_msg.data)
        self.intent = self.intent_dict["intent"]
        intent = self.intent

        self.blackboard.set(self.intent, True)

        if(self.publishers_dict[intent] and self.intent_dict.get('input')):
            goal = brain.BrainGoal()
            goal.goal=self.intent_dict['input']
            self.publishers_dict[intent].publish(goal)
    
    def action_requested(self, action):
        return self.current_requested_intent == action

    def shutdown(self):
        rospy.loginfo("Shutting Down Brain Node")
        #self.robot.run_action("stop",{})

if __name__ == '__main__':
    rospy.init_node('brain',anonymous=False)
    node = Brain()
    rate = rospy.Rate(1)

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