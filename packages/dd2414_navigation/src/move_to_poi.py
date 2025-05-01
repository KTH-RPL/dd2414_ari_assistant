#!/usr/bin/env python3

import rospy

import actionlib
from dd2414_status_update import StatusUpdate
from pal_composite_navigation_msgs.msg import GoToFloorPOIAction, GoToFloorPOIGoal
from visualization_msgs.msg import InteractiveMarkerInit
import json
import os
import numpy as np
import dd2414_brain_v2.msg as brain

class MoveToPOI:
    def __init__(self):
        self.string_header = "[MOVE_TO_POI    ]:"
        rospy.loginfo(self.string_header + "Initializing")
        self.poi_dict = {}
        self._ac_navigation = actionlib.SimpleActionClient('/composite_navigation',GoToFloorPOIAction)
        self._sub_map_poi = rospy.Subscriber('/poi_marker_server/update_full',InteractiveMarkerInit, self.map_poi_conversion)
        self.encodings_file = "/home/pal/deployed_ws/lib/dd2414_human_detection/face_database.json"

        self.current_goal = None
        self.status = 3
    
    def load_known_faces(self):
        if os.path.exists(self.encodings_file):
            with open(self.encodings_file, "r") as f:
                try:
                    data = json.load(f)
                    rospy.logdebug("[FACERECOGNITION]:Loaded face database successfully.")
                    
                    data["encodings"] = {key: [np.array(encoding, dtype=np.float64) for encoding in encodings] 
                                     for key, encodings in data["encodings"].items()}
                    return data
                except json.JSONDecodeError as e:
                    rospy.logerr(f"Error decoding JSON: {e}")
        else:
            rospy.logwarn(f"Face database file {self.encodings_file} does not exist.")
        return {"encodings": {}, "ids": [], "names": [], "coordinates": [], "room": []}

    def go_to_poi(self,goal):
            
            #If the goal is new we setup and send the goal
            if self.status != 0 and self.status != 1 :
                meta = GoToFloorPOIGoal()
                meta.poi = goal #This is assigning the waypoint for the GoToFloorPOI GOal
                self._ac_navigation.send_goal(meta)
            
            #We ask for updates of the Action Server
            status = self._ac_navigation.get_state()
            self.status = status
            result = self._ac_navigation.get_result()
            
            rospy.logdebug(self.string_header + str(status))
            rospy.logdebug(self.string_header + str(result))

            if status == 3:
                self.current_goal = None
                return "Success"
            elif status == 0 or status == 1:
                return "Working"
            else:
                self.current_goal = None
                return "Failure"

    def action(self,goal):
        result = brain.BrainResult()
        #If the goal has been changed
        if self.current_goal != goal.goal and self.current_goal is not None :
            result.result = "Failure"
            self.preempted()

        else: #If the goal is the same or its a new goal
            #Check if the goal is a POI
            if goal.goal in self.poi_dict:
                result.result=self.go_to_poi(goal.goal)
            #If the goal is not a POI but a Person
            else:
                room = self.get_poi_from_person(goal.goal)

                #If there is a registered room to the person
                if room is not None and room != "" :
                    result.result= self.go_to_poi(room)
                else:
                    if room == "":
                        rospy.loginfo(self.string_header + "Person is not registered to any room: " + goal.goal)
                    else:
                        rospy.loginfo(self.string_header + "Waypoint not found: " + goal.goal)
                    result.result = "Failure"

        self.current_goal = goal.goal

        return result
    
    def preempted(self):
        rospy.loginfo(self.string_header + "Cancelling All Goals")
        self._ac_navigation.cancel_all_goals()
        pass
        
    def get_poi_from_person(self,goal):
        try:
            index = self.known_face["names"].index(goal)
        except ValueError:
            return None

        # Obtain name
        return self.known_face["room"][index]

    def map_poi_conversion(self,data):
        marker_dict = {marker.name : marker for marker in data.markers}
        self.poi_dict = marker_dict
        rospy.logdebug(self.string_header + str(list(marker_dict.keys())))

        self.known_face = self.load_known_faces() 
if __name__ == '__main__':
    rospy.init_node('move_to_poi',log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(),MoveToPOI)

    rospy.spin()