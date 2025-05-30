#!/usr/bin/env python3

import rospy
import actionlib
from dd2414_status_update import StatusUpdate
import dd2414_brain_v2.msg as brain
from datetime import datetime
import py_trees

class ExploreBehaviour(py_trees.behaviour.Behaviour):    
    def __init__(self, name = "explore"):
        super(ExploreBehaviour, self).__init__(name)
        self.current_goal = None
        rospy.loginfo("[EXPLORE        ]: Initialized")
        self.string_header = "[EXPLORE        ]:"

        self.rooms = ["copy_room", "kitchen", "office"]

        self.room_index = 0
        self.client = actionlib.SimpleActionClient("/move_to_poi", brain.BrainAction)
        self.sent_goal = False
        self.timeout = rospy.Duration(10)
    
    def initialise(self):
        self.sent_goal = False
        self.room_index = 0
        self.prioritized_rooms = self.get_prioritized_rooms()
        return True

    def update(self):
        if not self.client.wait_for_server(self.timeout):
            rospy.logerr(f"{self.string_header} Navigation server not available")
            return py_trees.common.Status.FAILURE

        if self.room_index >= len(self.prioritized_rooms):
            return py_trees.common.Status.SUCCESS

        current_room = self.prioritized_rooms[self.room_index]

        if not self.sent_goal:
            goal = brain.BrainGoal()
            goal.goal = current_room
            self.client.send_goal(goal)
            self.sent_goal = True
            rospy.loginfo(f"{self.string_header} Going to {current_room}")
            self.starting_time = rospy.Time.now()

        state = self.client.get_state()
        rospy.logdebug(f"{self.string_header} The state of the Go To Service is: {state}")
        current_time = rospy.Time.now()

        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"{self.string_header} Reached {current_room}")
            self.room_index += 1
            self.sent_goal = False
            return py_trees.common.Status.RUNNING
        elif state in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE] and (current_time-self.starting_time < rospy.Duration(40)):
            return py_trees.common.Status.RUNNING
        else:
            rospy.logwarn(f"{self.string_header} Failed to reach {current_room}, skipping")
            self.room_index += 1
            self.sent_goal = False
            return py_trees.common.Status.RUNNING
        
    def preempted(self):
        # Procedure in case the call gets cancelled
        print("PREEMPTED EXPLORATION")
        self.client.cancel_all_goals()
        pass
        
    def get_prioritized_rooms(self):
        current_hour = datetime.now().hour
        prioritized = []

        # Basic weights
        weights = {room: 1 for room in self.rooms}

        # Priority to kitchen between 12 and 13
        if 12 <= current_hour < 13:
            weights["kitchen"] += 2

        ### Maybe add dynamic priorities based on local costmap


        # Sort by weight
        prioritized = sorted(self.rooms, key=lambda room: -weights[room])

        rospy.loginfo(f"{self.string_header} Room priorities: {prioritized}")
        return prioritized

if __name__ == '__main__':
    rospy.init_node('exploration_node', anonymous=False,log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(), ExploreBehaviour)
    rospy.spin()