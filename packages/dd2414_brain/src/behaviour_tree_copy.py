#!/usr/bin/env python3


import operator
import os
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from py_trees.display import render_dot_tree
from stop_behaviour import StopBehaviour
from exploration_node import ExploreBehaviour
from rospy.exceptions import ROSException
import dd2414_brain.msg as brain
from dd2414_brain.msg import BrainAction 
from py_trees.common import VisibilityLevel

import actionlib

import py_trees
import py_trees_ros


#Dont MOVE ANYTING FROM HERE
class Brain:
    def __init__(self):

        rospy.loginfo("Brain Start Initializing")

        self.intent_sub=rospy.Subscriber(rospy.get_param("~IN_intent_topic","~intent"),String, self.intent_cb, queue_size=1)

        self.intent_dict = {}
        self.intent = ""

        self.blackboard = py_trees.blackboard.Blackboard()
        self.current_requested_intent = None

        self.namespace_dict = {
            "stop"                 :"/stop",
            "translate"            :"/translate_conversation",
            "move to translate"    :"/body_orientation_listener",
            "remember user"        :"/face_recognition_node",
            "face recognition"     :"/face_recognition_node",
            "go to"                :"/move_to_poi",
            "find speaker"         :"/find_speaker",
            "follow user"          :"/follow_user",
            #"move to person"       :"/body_orientation_listener",
            "provide information"  :"/ollama_response",
            #"speech"               :self.text_to_speech,
            "greet"                :"/face_recognition_node",
            "goodbye"              :"/face_recognition_node",
            "explore"              :"/explore",
            "reject"               :"/ollama_response"
        }

        self.namespace_dict = {
            "stop"                 :"/translate_conversation",
            "translate"            :"/translate_conversation",
            "move to translate"    :"/translate_conversation",
            "remember user"        :"/translate_conversation",
            "face recognition"     :"/translate_conversation",
            "go to"                :"/translate_conversation",
            "find speaker"         :"/translate_conversation",
            "follow user"          :"/translate_conversation",
            #"move to person"       :"/body_orientation_listener",
            "provide information"  :"/translate_conversation",
            #"speech"               :self.text_to_speech,
            "greet"                :"/translate_conversation",
            "goodbye"              :"/translate_conversation",
            "explore"              :"/translate_conversation",
            "reject"               :"/translate_conversation"
        }


        self.behaviours = {}
        self.publishers_dict = {}

        for action in self.namespace_dict:
            try:
                
                behaviour = py_trees_ros.actions.ActionClient(
                    name=action,
                    action_namespace=self.namespace_dict[action],
                    action_spec=brain.BrainAction,
                    action_goal=brain.BrainGoal())
                self.behaviours[action] = behaviour

                publisher = rospy.Publisher(f"/brain{self.namespace_dict[action]}", brain.BrainGoal, queue_size=1)
                self.publishers_dict[action] = publisher
            except ROSException as e:
                rospy.logwarn(f"Could not connect to {self.namespace_dict[action]}")

        [look_at_face_behaviour, stop_look_at_face_behaviour] = self.setup_look_at_face()
        
        
        go_to_behaviour = py_trees.Sequence(
            "Stop looking at person, go to",
            [stop_look_at_face_behaviour, 
             self.behaviours['go to']]#, look_at_face_behaviour]
        )

        follow_user_behaviour = py_trees.Sequence(
            "Find speaker, then follow user", 
            #[stop_look_at_face_behaviour,
            [self.behaviours["find speaker"],  
             self.behaviours["follow user"]]
             #look_at_face_behaviour]
             )
        
        stop_behaviour = py_trees.Sequence(
            "Stop, look at person",
            [StopBehaviour(name="stop behaviour", action_dict=self.namespace_dict), 
             look_at_face_behaviour]
        )


        self.action_dict = {
            "stop"                 :stop_behaviour,
            "translate"            :self.setup_translate_behaviour(),
            #"remember user"        :self.setup_greet_behaviour(),
            #"face recognition"     :self.behaviours["face recognition"],
            "go to"                :go_to_behaviour,
            #"find speaker"         :self.behaviours['find speaker'],
            "follow user"          :follow_user_behaviour,
            "provide information"  :self.behaviours['provide information'],
            #"speech"               :self.text_to_speech,
            "greet"                :self.setup_greet_behaviour(),
            "goodbye"              :self.setup_greet_behaviour(),
            "explore"              :ExploreBehaviour(name = "explore"),
            "reject"               :self.behaviours['reject']
            }      

        for action in self.action_dict:
            self.blackboard.set(action, False)
            self.blackboard.set(f"failcounter {action}", 0)

        # Set up behaviours and construct tree
        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        # For every action, set up condition (action requested) and behaviour (action)
        for action in self.action_dict:

            if(not self.service_is_available(self.namespace_dict[action]) and not action=="stop" and not action=="explore"):
                rospy.logwarn(f"Service {self.namespace_dict[action]} not available, skipping {action} behaviour")
                #root.add_child(py_trees.behaviours.Success(name=f"Mock {action}"))
                continue
            
            behaviour = self.action_dict[action]

            # Create condition to check if action is requested
            condition = py_trees.blackboard.CheckBlackboardVariable(
                name=f"{action} action not requested?",
                variable_name=action,
                expected_value=False
            )
            condition.visibility_level = py_trees.common.VisibilityLevel.DETAIL 

            reset_fail_counter = py_trees.blackboard.SetBlackboardVariable(
                name="Set fail counter back to 0",
                variable_name=f"failcounter {action}",
                variable_value=0
            )
            reset_fail_counter.visibility_level = py_trees.common.VisibilityLevel.DETAIL 

            set_action_requested_to_false = py_trees.blackboard.SetBlackboardVariable(
                name="Set Action requested back to false",
                variable_name=action,
                variable_value=False
            )
            set_action_requested_to_false.visibility_level = py_trees.common.VisibilityLevel.DETAIL 

            check_fail_counter = py_trees.blackboard.CheckBlackboardVariable(
                name=f"{action} failcounter more than 10?",
                variable_name=f"failcounter {action}",
                expected_value=10,
                comparison_operator=operator.ge,
                debug_feedback_message=True
            )
            check_fail_counter.visibility_level = py_trees.common.VisibilityLevel.DETAIL 

            counter_check = py_trees.composites.Sequence(
                f"Check fail counter",
                [
                check_fail_counter,
                set_action_requested_to_false,
                reset_fail_counter
                ]
            )
            counter_check.visibility_level = py_trees.common.VisibilityLevel.DETAIL 

            # Create behaviour to be executed if action is requested
            

            incrementor = py_trees.behaviours.Failure(name="IncrementCounter")
            incrementor.update = (lambda a=action: self.increment_fail_counter(a) or py_trees.common.Status.FAILURE)
            incrementor.visibility_level = py_trees.common.VisibilityLevel.DETAIL 

            execute_and_set_req_to_false = py_trees.composites.Sequence(
                            f"Execute {action} action and set action requested to false", 
                            [
                            py_trees.composites.Selector(
                                f"Execute {action} action or increase counter",
                                [behaviour,
                                 incrementor]

                            ), 
                            set_action_requested_to_false,
                            reset_fail_counter
                            ])
            #execute_and_set_req_to_false.visibility_level = py_trees.common.VisibilityLevel.DETAIL 
            # Add condition and behaviour to tree
            root.add_children([
                py_trees.composites.Selector(
                    f"Intent {action}", 
                    [
                        behaviour
                    ]),
                ])

        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)
        print(py_trees.display.print_ascii_tree(root=self.behaviour_tree.root))        

        self.behaviour_tree.setup(timeout=10)

        print("hellohellohello")
        look_at_face_behaviour.tick_once()
        rospy.loginfo("Brain Finished Initializing")
        level = py_trees.common.VisibilityLevel.BIG_PICTURE
        filtered_tree = self.filter_tree_by_visibility_level(self.behaviour_tree.root)
        tree = py_trees.display.render_dot_tree(self.behaviour_tree.root, visibility_level=level)

        print("sleeping")
        self.destroy_node() 
        rospy.sleep(100)

    def print_tree(self, tree: py_trees.trees.BehaviourTree) -> None:
        # Print the behaviour tree and its current status.
        print(py_trees.display.print_ascii_tree(root=tree.root, show_status=True))

    def increment_fail_counter(self, action):
        current = self.blackboard.get(f"failcounter {action}")
        self.blackboard.set(f"failcounter {action}", current + 1)

    def intent_cb(self,string_msg):

        self.intent_dict = json.loads(string_msg.data)
        self.intent = self.intent_dict["intent"]
        intent = self.intent

        self.blackboard.set(self.intent, True)


        if(self.publishers_dict[intent] and self.intent_dict.get('input')):
            goal = brain.BrainGoal()
            goal.goal=self.intent_dict['input']
            goal.in_dic = json.dumps(self.intent_dict)


            self.publishers_dict[intent].publish(goal)
    
    def setup_look_at_face(self):
        if(self.service_is_available("/translate_conversation")):
            look_at_face_goal = brain.BrainGoal()
            look_at_face_goal.goal = "start"
            look_at_face_behaviour = py_trees_ros.actions.ActionClient(
                    name="look at face",
                    action_namespace="/translate_conversation",
                    action_spec=brain.BrainAction,
                    action_goal=look_at_face_goal)
            
            stop_look_at_face_goal = brain.BrainGoal()
            stop_look_at_face_goal.goal = "stop"
            stop_look_at_face_behaviour = py_trees_ros.actions.ActionClient(
                    name="stop look at face",
                    action_namespace="/translate_conversation",
                    action_spec=brain.BrainAction,
                    action_goal=stop_look_at_face_goal)
            return [look_at_face_behaviour, stop_look_at_face_behaviour]
        else:
            look_at_face_behaviour = py_trees.behaviours.Success(name="Mock look at face")
            stop_look_at_face_behaviour = py_trees.behaviours.Success(name="Mock stop look at face")
            return [look_at_face_behaviour, stop_look_at_face_behaviour]
        
    def setup_greet_behaviour(self):
        [look_at_face_behaviour, stop_look_at_face_behaviour] = self.setup_look_at_face()
        
        find_speaker = "find speaker"
        find_speaker_behaviour = py_trees_ros.actions.ActionClient(
                    name=find_speaker,
                    action_namespace=self.namespace_dict[find_speaker],
                    action_spec=brain.BrainAction,
                    action_goal=brain.BrainGoal())
        
        face_recognition = "face recognition"
        face_recognition_behaviour = py_trees_ros.actions.ActionClient(
                    name=face_recognition,
                    action_namespace=self.namespace_dict[face_recognition],
                    action_spec=brain.BrainAction,
                    action_goal=brain.BrainGoal())

        return py_trees.Sequence(
            "Find speaker, then say hello (with name)", 
            [stop_look_at_face_behaviour,
             find_speaker_behaviour, 
             face_recognition_behaviour,
             look_at_face_behaviour])
    
    # Move to translate not working
    def setup_translate_behaviour(self):
        [look_at_face_behaviour, stop_look_at_face_behaviour] = self.setup_look_at_face()

        find_speaker = "find speaker"
        find_speaker_behaviour = py_trees_ros.actions.ActionClient(
                    name=find_speaker,
                    action_namespace=self.namespace_dict[find_speaker],
                    action_spec=brain.BrainAction,
                    action_goal=brain.BrainGoal())
        
        move_to_translate = "move to translate"
        move_to_translate_behaviour = py_trees_ros.actions.ActionClient(
                    name=move_to_translate,
                    action_namespace=self.namespace_dict[move_to_translate],
                    action_spec=brain.BrainAction,
                    action_goal=brain.BrainGoal())
        
        translate = "translate"
        translate_behaviour = py_trees_ros.actions.ActionClient(
                    name=translate,
                    action_namespace=self.namespace_dict[translate],
                    action_spec=brain.BrainAction,
                    action_goal=brain.BrainGoal())
        
        return py_trees.Sequence(
            "Find speaker, then move next to them and translate", 
            [stop_look_at_face_behaviour,
             find_speaker_behaviour, 
             move_to_translate_behaviour,
             translate_behaviour])

    def action_requested(self, action):
        return self.current_requested_intent == action

    def shutdown(self):
        rospy.loginfo("Shutting Down Brain Node")

    def service_is_available(self, name, timeout=1.0):
        return True
        client = actionlib.SimpleActionClient(name, brain.BrainAction)
        return client.wait_for_server(timeout=rospy.Duration(timeout))

    def filter_tree_by_visibility_level(self, root, level=VisibilityLevel.BIG_PICTURE):

        # Default to BIG_PICTURE if not explicitly set
        node_level = getattr(root, 'visibility_level', VisibilityLevel.BIG_PICTURE)
        print(f"node_level {node_level}")
        
        

        # Clone the node (shallow copy)
        print(f"node name {root.name}")
        if(root.name == "stop behaviour" or root.name == "Execute stop action or increase counter"):
            print("123")
            new_node = type(root)(name=root.name, action_dict=self.namespace_dict)
        else:
            new_node = type(root)(name=root.name)
        new_node.visibility_level = node_level  # Preserve the tag

        filtered_child = None
        # Recursively filter and add children (if composite)
        if isinstance(root, py_trees.composites.Composite):
            for child in root.children:
                filtered_child = self.filter_tree_by_visibility_level(child, level)
                if filtered_child is not None:
                    new_node.add_child(filtered_child)
            

        # If this node is not visible enough, skip it
        if node_level < level:
            if filtered_child is not None:
                print("-------------returning child")
                return filtered_child
            return None
        
        
        return new_node


if __name__ == '__main__':
    rospy.init_node('brain1',anonymous=False)
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