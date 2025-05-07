#!/usr/bin/env python3

import rospy
import actionlib
from dd2414_status_update import StatusUpdate
import dd2414_brain_v2.msg as brain
from datetime import datetime

class ExplorationNode:
    def __init__(self):
        self.current_goal = None
        rospy.loginfo("[EXPLORE        ]: Initialized")
        self.string_header = "[EXPLORE        ]:"

        self.rooms = ["mo_cap", "kitchen", "Inner_Office", "copy_room"]

    def action(self, goal):
        result = brain.BrainResult()

        rospy.loginfo(f"{self.string_header} Starting exploration.")

        # Prioritize rooms for this round
        prioritized_rooms = self.get_prioritized_rooms()        
        



    def get_prioritized_rooms(self):
        current_hour = datetime.now().hour
        prioritized = []

        # Basic weights
        weights = {room: 1 for room in self.rooms}

        # Priority to kitchen between 12 and 13
        if 12 <= current_hour < 13:
            weights["kitchen"] += 2

        ###################################################### Maybe add dynamic priorities based on local costmap


        # Sort by weight
        prioritized = sorted(self.rooms, key=lambda room: -weights[room])

        rospy.loginfo(f"{self.string_header} Room priorities: {prioritized}")
        return prioritized

if __name__ == '__main__':
    rospy.init_node('exploration_node', anonymous=False,log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(), ExplorationNode)
    rospy.spin()
