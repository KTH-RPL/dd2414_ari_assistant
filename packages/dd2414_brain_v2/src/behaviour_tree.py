#!/usr/bin/env python3


import operator
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
import time
from stop_behaviour import StopBehaviour
from exploration_node import ExploreBehaviour
from rospy.exceptions import ROSException
import dd2414_brain_v2.msg as brain
from dd2414_brain_v2.msg import BrainAction 

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
            "explore"              :"/explore"
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
            [stop_look_at_face_behaviour, self.behaviours['go to'], look_at_face_behaviour]
        )

        follow_user_behaviour = py_trees.Sequence(
            "Find speaker, then follow user", 
            [self.behaviours["find speaker"],  
             self.behaviours["follow user"],
             look_at_face_behaviour])
        
        greet_behaviour = py_trees.Sequence(
            "Find speaker, then say hello (with name)", 
            [self.behaviours["find speaker"], 
             self.behaviours["face recognition"],
             look_at_face_behaviour])
        
        remember_user_behaviour = py_trees.Sequence(
            "Find speaker, then remember their name", 
            [self.behaviours["find speaker"],  
             self.behaviours["face recognition"],
             look_at_face_behaviour])
        
        goodbye_behaviour = py_trees.Sequence(
            "Find speaker, and tell them goodbye (with name)", 
            [self.behaviours["find speaker"],  
             self.behaviours["face recognition"],
             look_at_face_behaviour])
        

        
        translate_behaviour = self.behaviours["translate"]


        self.action_dict = {
            "stop"                 :StopBehaviour(name="stop behaviour", action_dict=self.namespace_dict),
            "translate"            :translate_behaviour,
            "remember user"        :remember_user_behaviour,
            "face recognition"     :self.behaviours["face recognition"],
            "go to"                :go_to_behaviour,
            "find speaker"         :self.behaviours['find speaker'],
            "follow user"          :follow_user_behaviour,
            "provide information"  :self.behaviours['provide information'],
            #"speech"               :self.text_to_speech,
            "greet"                :greet_behaviour,
            "goodbye"              :goodbye_behaviour,
            "explore"              :ExploreBehaviour(name = "explore behaviour", action_dict = self.namespace_dict)
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
                expected_value=False,
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
            # Create behaviour to be executed if action is requested
            

            incrementor = py_trees.behaviours.Failure(name="IncrementCounter")
            incrementor.update = (lambda a=action: self.increment_fail_counter(a) or py_trees.common.Status.FAILURE)


            # Add condition and behaviour to tree
            root.add_children([
                py_trees.composites.Selector(
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
                    ]),
                ])

        self.behaviour_tree = py_trees.trees.BehaviourTree(root=root)
        print(py_trees.display.print_ascii_tree(root=self.behaviour_tree.root))        

        self.behaviour_tree.setup(timeout=10)

        look_at_face_behaviour.tick_once()
        rospy.loginfo("Brain Finished Initializing")

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
            if intent == "translate":
                goal.goal=(self.intent_dict['input']).replace("\"","")
                goal.in_dic = json.dumps(self.intent_dict["language"])
            else:
                goal.goal=self.intent_dict['input']
                goal.in_dic = json.dumps(self.intent_dict)


            self.publishers_dict[intent].publish(goal)
    
    def setup_look_at_face(self):
        if(self.service_is_available("/face_gaze_tracker")):
            look_at_face_goal = brain.BrainGoal()
            look_at_face_goal.goal = "start"
            look_at_face_behaviour = py_trees_ros.actions.ActionClient(
                    name="Look at face",
                    action_namespace="face_gaze_tracker",
                    action_spec=brain.BrainAction,
                    action_goal=look_at_face_goal)
            
            stop_look_at_face_goal = brain.BrainGoal()
            stop_look_at_face_goal.goal = "stop"
            stop_look_at_face_behaviour = py_trees_ros.actions.ActionClient(
                    name="Stop look at face",
                    action_namespace="face_gaze_tracker",
                    action_spec=brain.BrainAction,
                    action_goal=stop_look_at_face_goal)
            return [look_at_face_behaviour, stop_look_at_face_behaviour]
        else:
            look_at_face_behaviour = py_trees.behaviours.Success(name="Mock look at face")
            stop_look_at_face_behaviour = py_trees.behaviours.Success(name="Mock stop look at face")
            return [look_at_face_behaviour, stop_look_at_face_behaviour]

    def action_requested(self, action):
        return self.current_requested_intent == action

    def shutdown(self):
        rospy.loginfo("Shutting Down Brain Node")

    def service_is_available(self, name, timeout=1.0):
        client = actionlib.SimpleActionClient(name, brain.BrainAction)
        return client.wait_for_server(timeout=rospy.Duration(timeout))

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