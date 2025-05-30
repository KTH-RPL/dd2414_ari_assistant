#!/usr/bin/env python3

import rospy

import actionlib
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from dd2414_status_update import StatusUpdate
from pal_navigation_msgs.msg import GoToPOIAction, GoToPOIGoal
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction
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
        self.goal_id = ""
        self._ac_navigation = actionlib.SimpleActionClient('/poi_navigation_server/go_to_poi',GoToPOIAction)
        self.move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.ollama_response_client = actionlib.SimpleActionClient("/ollama_response",brain.BrainAction)
        self._sub_map_poi = rospy.Subscriber('/poi_marker_server/update_full',InteractiveMarkerInit, self.map_poi_conversion)
        self.encodings_file = "/home/pal/deployed_ws/lib/dd2414_human_detection/face_database.json"
        self.current_room = ""
        self.status = -1
    

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
                    rospy.logerr(f"{self.string_header}Error decoding JSON: {e}")
        else:
            rospy.logwarn(f"[MOVE_TO_POI    ]:Face database file {self.encodings_file} does not exist.")

        return {"encodings": {}, "ids": [], "names": [], "coordinates": [], "room": []}

    def go_to_poi(self,goal):
            
            #If the goal is new we setup and send the goal
            if self.status != 0 and self.status != 1 :
                meta = GoToPOIGoal()
                meta.poi.data = goal #This is assigning the waypoint for the GoToFloorPOI GOal

                self._ac_navigation.wait_for_server(rospy.Duration(10))
                self._ac_navigation.send_goal(meta)
                #self.goal_id = self.goal_id_recieved

            
            #We ask for updates of the Action Server
            status = self._ac_navigation.get_state()
            self.status = status
            result = self._ac_navigation.get_result()
            
            rospy.logdebug(self.string_header +f"Status:{status}")
            rospy.logdebug(self.string_header +f"Result:{result}")

            if status == 3:
                self.goal_id = ""
                return "Success"

            elif status == 0  or status == 1:
                return "Working"
            else:
                self.goal_id = ""
                return "Failure"

    def action(self,goal):
        result = brain.BrainResult()
        #rospy.loginfo(self.string_header + "Action Being Called: " + str(goal))
        #If the goal is the same or its a new goal
        #Check if the goal is a POI
        if goal.goal in self.poi_dict:
            result.result=self.go_to_poi(goal.goal)
        #If the goal is not a POI but a Person
        else:
            room = self.get_poi_from_person(goal.goal)
            if self.current_room != room: #Room the person is has been changed
                if self.status == 0 or self.status == 1:
                    rospy.loginfo(self.string_header + f"The persons location has been updated | Status :{self.status} |Current Room: {self.current_room} | Room: {room}")
                    self.preempted()
                    rospy.loginfo(f"{self.string_header} Found the person \"{goal.goal}\" in room \"{room}\" when it was this room before \"{self.current_room}\"")
                    goal = brain.BrainGoal()
                    goal.goal = "found_person"
                    goal.in_dic = json.dumps({"intent":"","input":"","phrase":"Found the person","language":"en"})
                    self.ollama_response_client.send_goal(goal)
                    self.status = 3
                    result.result = "Success"
                    return result
            self.current_room = room


            #If there is a registered room to the person
            if room is not None and room != "" :
                rospy.logdebug(f"{self.string_header}Going to Person in {room}")
                result.result= self.go_to_poi(room)
            else:
                if room == None:
                    rospy.loginfo(self.string_header + "Person is not registered to any room: " + goal.goal)
                else:
                    rospy.loginfo(self.string_header + "Waypoint not found: " + goal.goal)
                result.result = "Failure"


        return result
    

    def preempted(self):
        rospy.loginfo(self.string_header + "Cancelling All Goals")
        #Cancel the POI SERVER GOAL, this will cancel one goal in the MOVE BASE action server
        #But a new one will pop up after detecting the MOVE BASE goal has been preempted
        #So we need to wait for the 2nd one to pop and then we cancell that goal, in this case we are cancelling all goals
        self._ac_navigation.cancel_all_goals()
        rospy.sleep(rospy.Duration(0.5))
        self.move_client.cancel_all_goals()
        self.status = 4
        self.current_room = ""
        return
        
    def get_poi_from_person(self,goal):
        try:
            index = self.known_face["names"].index(goal)
        except ValueError:
            return None

        # Obtain name
        room =self.known_face["room"][index]
        return room 

    def map_poi_conversion(self,data):
        marker_dict = {marker.name : marker for marker in data.markers}
        self.poi_dict = marker_dict
#        rospy.logdebug(self.string_header + str(list(marker_dict.keys())))

        self.known_face = self.load_known_faces() 
if __name__ == '__main__':
    rospy.init_node('move_to_poi',log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(),MoveToPOI)

    rospy.spin()