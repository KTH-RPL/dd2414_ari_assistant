#!/usr/bin/env python3


import operator
import os
import rospy
import json
from std_msgs.msg import String
from stop_behaviour import StopBehaviour
from exploration_node import ExploreBehaviour
from rospy.exceptions import ROSException
import dd2414_brain_v2.msg as brain

import actionlib

import py_trees
import py_trees_ros


class Brain:
    def __init__(self):

        rospy.loginfo("Brain Start Initializing")

        self.intent_sub=rospy.Subscriber(rospy.get_param("~IN_intent_topic","~intent"),String, self.intent_cb, queue_size=1)
        self.blackboard = py_trees.blackboard.Blackboard()

        # Namespaces for action servers (if connected to an intent, name should be the same)
        self.namespace_dict = {
            "stop"                 :"/ollama_response",
            "translate"            :"/translate_conversation",
            "move to translate"    :"/body_orientation_listener",
            "remember user"        :"/face_recognition_node",
            "face recognition"     :"/face_recognition_node",
            "go to"                :"/move_to_poi",
            "find speaker"         :"/find_speaker",
            "follow user"          :"/follow_user",
            "provide information"  :"/ollama_response",
            "greet"                :"/face_recognition_node",
            "goodbye"              :"/face_recognition_node",
            "explore"              :"/ollama_response",
            "reject"               :"/ollama_response",
            "look at person"       :"/face_gaze_tracker",
            "find object"          :"/move_to_poi"
        }

        # Set up publishers for goals
        self.publishers_dict = {}

        for action in self.namespace_dict:
            try:
                publisher = rospy.Publisher(f"/brain{self.namespace_dict[action]}", brain.BrainGoal, queue_size=1)
                self.publishers_dict[action] = publisher
            except ROSException as e:
                rospy.logwarn(f"Could not connect to {self.namespace_dict[action]}")

        # Set up more complex behaviours
        [look_at_face_behaviour, stop_look_at_face_behaviour] = self.setup_look_at_face()
        go_to_behaviour = py_trees.Sequence(
            "Stop looking at person, go to",
            [stop_look_at_face_behaviour, self.create_behaviour_from_action_server("go to"), look_at_face_behaviour]
        )

        follow_user_behaviour = py_trees.Sequence(
            "Find speaker, then follow user", 
            [self.create_behaviour_from_action_server("find speaker"),  
             self.create_behaviour_from_action_server("follow user")]
             )
        
        stop_behaviour = py_trees.Sequence(
            "Stop, look at person",
            [StopBehaviour(name="stop behaviour", action_dict=self.namespace_dict), 
             self.create_behaviour_from_action_server("stop"),
             look_at_face_behaviour]
        )

        translate_behaviour = py_trees.Sequence(
            "Find speaker, then move next to them and translate", 
            [stop_look_at_face_behaviour,
             #self.create_behaviour_from_action_server("find speaker"),
             #self.create_behaviour_from_action_server("move to translate"),
             self.create_behaviour_from_action_server("translate")])

        explore_behaviour = py_trees.Sequence(
            "Explore", 
            [self.create_behaviour_from_action_server("explore"),
             ExploreBehaviour(name = "explore"),
             ])

        # Define behaviours for each intent
        self.action_dict = {
            "stop"                 :stop_behaviour,
            "translate"            :translate_behaviour,
            "remember user"        :self.setup_greet_behaviour(),
            "go to"                :go_to_behaviour,
            "find object"          :go_to_behaviour,
            "follow user"          :follow_user_behaviour,
            "provide information"  :self.create_behaviour_from_action_server("provide information"),
            "greet"                :self.setup_greet_behaviour(),
            "goodbye"              :self.setup_greet_behaviour(),
            "explore"              :explore_behaviour,
            "reject"               :self.create_behaviour_from_action_server("reject")
            }      

        # Set up the failcounters and action requests in the blackboard
        for action in self.action_dict:
            self.blackboard.set(action, False)
            self.blackboard.set(f"failcounter {action}", 0)

        # Construct tree
        root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        for action in self.action_dict:

            if(not self.service_is_available(self.namespace_dict[action]) and not action=="stop" and not action=="explore"):
                rospy.logwarn(f"Service {self.namespace_dict[action]} not available, skipping {action} behaviour")
                continue

            root.add_child(self.intent_to_behaviour_tree(action))            

        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)
        print(py_trees.display.print_ascii_tree(root=self.behaviour_tree.root))        
        self.behaviour_tree.setup(timeout=10)

        # Start looking at faces
        look_at_face_behaviour.tick_once()

        rospy.loginfo("Brain Finished Initializing")

    def print_tree(self, tree: py_trees.trees.BehaviourTree) -> None:
        # Print the behaviour tree and its current status.
        print(py_trees.display.print_ascii_tree(root=tree.root, show_status=True))

    def increment_fail_counter(self, action):
        current = self.blackboard.get(f"failcounter {action}")
        self.blackboard.set(f"failcounter {action}", current + 1)

    def intent_cb(self,string_msg):

        intent_dict = json.loads(string_msg.data)
        intent = intent_dict["intent"]

        self.blackboard.set(intent, True)

        if(self.publishers_dict[intent] and intent_dict.get('input')):
            goal = brain.BrainGoal()
            goal.goal=intent_dict['input']
            goal.in_dic = json.dumps(intent_dict)

            self.publishers_dict[intent].publish(goal)
        
    def create_behaviour_from_action_server(self, action, name = None, goal = None):
        brainGoal = brain.BrainGoal()
        if(goal is not None):
            brainGoal.goal = goal

        behaviourName = action 
        if(name is not None):
            behaviourName = name

        behaviour = py_trees_ros.actions.ActionClient(
            name=behaviourName,
            action_namespace=self.namespace_dict[action],
            action_spec=brain.BrainAction,
            action_goal=brainGoal)
        
        return behaviour

    def setup_greet_behaviour(self):
        [look_at_face_behaviour, stop_look_at_face_behaviour] = self.setup_look_at_face()

        find_speaker_behaviour = self.create_behaviour_from_action_server("find speaker")
        face_recognition_behaviour = self.create_behaviour_from_action_server("face recognition")

        return py_trees.Sequence(
            "Find speaker, then say hello (with name)", 
            [stop_look_at_face_behaviour,
             find_speaker_behaviour, 
             face_recognition_behaviour,
             look_at_face_behaviour])
    
    def setup_look_at_face(self):
        if(self.service_is_available(self.namespace_dict["look at person"])):
            look_at_face = self.create_behaviour_from_action_server("look at person", goal = "start")
            
            stop_look_at_face = self.create_behaviour_from_action_server("look at person", name="stop look at person", goal = "stop")
            
            return [look_at_face, stop_look_at_face]
        
        # If look at face is not available, create mock behaviours so tree doesn't crash
        else:
            look_at_face = py_trees.behaviours.Success(name="Mock look at face")
            stop_look_at_face = py_trees.behaviours.Success(name="Mock stop look at face")
            return [look_at_face, stop_look_at_face]
    
    def intent_to_behaviour_tree(self, action):
        
        behaviour = self.action_dict[action]

        # Create condition to check if action is requested
        condition = py_trees.blackboard.CheckBlackboardVariable(
            name=f"{action} action not requested?",
            variable_name=action,
            expected_value=False
        )

        reset_fail_counter = py_trees.blackboard.SetBlackboardVariable(
            name="Set fail counter back to 0",
            variable_name=f"failcounter {action}",
            variable_value=0
        )

        set_action_requested_to_false = py_trees.blackboard.SetBlackboardVariable(
            name="Set Action requested back to false",
            variable_name=action,
            variable_value=False
        )

        check_fail_counter = py_trees.blackboard.CheckBlackboardVariable(
                name=f"{action} failcounter more than 10?",
                variable_name=f"failcounter {action}",
                expected_value=10,
                comparison_operator=operator.ge,
                debug_feedback_message=True
            )

        counter_check = py_trees.composites.Sequence(
            f"Check fail counter",
            [
            check_fail_counter,
            set_action_requested_to_false,
            reset_fail_counter
            ]
        )

        # Increment fail counter
        incrementor = py_trees.behaviours.Failure(name="IncrementCounter")
        incrementor.update = (lambda a=action: self.increment_fail_counter(a) or py_trees.common.Status.FAILURE)

        # Add condition and behaviour to tree
        return py_trees.composites.Selector(
                f"Execute {action} action if requested", 
                [
                    condition,
                    counter_check,
                    py_trees.composites.Sequence(
                        f"Execute {action} action and set action requested to false", 
                        [
                        py_trees.composites.Selector(
                            f"Execute {action} action or increase counter",
                            [behaviour,
                                incrementor]

                        ), 
                        set_action_requested_to_false,
                        reset_fail_counter
                        ]),
                ])  

    def shutdown(self):
        rospy.loginfo("Shutting Down Brain Node")

    def service_is_available(self, name, timeout=1.0):
        client = actionlib.SimpleActionClient(name, brain.BrainAction)
        return client.wait_for_server(timeout=rospy.Duration(timeout))

if __name__ == '__main__':
    rospy.init_node('brain',anonymous=False)
    node = Brain()
    rate = rospy.Rate(5)

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