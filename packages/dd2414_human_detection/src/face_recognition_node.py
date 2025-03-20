#!/usr/bin/env python3
import rospy
import os
import cv2
import face_recognition

from sensor_msgs.msg import Image
from hri_msgs.msg import IdsList
from pyhri import HRIListener
from cv_bridge import CvBridge
import json
import numpy as np
from dd2414_status_update import StatusUpdate
import dd2414_brain_v2.msg as brain



class FaceRecognitionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.hri_listener = HRIListener()

        self.target_name = None
        
        # Paths for saving data
        self.database_path = os.path.dirname(__file__)
        self.encodings_file = os.path.join(self.database_path, "face_database.json")
        os.makedirs(self.database_path, exist_ok=True)
        
        # Load known faces from the file
        self.known_faces = self.load_known_faces()
        
        # ROS subscribers
        self.face_ids_sub = rospy.Subscriber("/humans/faces/tracked", IdsList, self.face_id_callback)
        self.face_images_subs = {}

    def action(self,goal):
        """
        This function will execute when it receives a goal from the brain. It will contain a string (goal.goal)
        which indicates the name of the person we are currently seeing. 
        """
        result = brain.BrainActionResult()

        if goal.goal:
            self.target_name = goal.goal

            # Save name if its not unknown
            if self.target_name != "unknown":
                rospy.loginfo(f"Save name: {self.target_name}")
                self.add_name_to_face(self.target_name)

                # Return name that was saved
                result.dict = self.target_name

            
            else:
                rospy.loginfo(f"Name unknown. Searching if we already know it.")
                name = self.search_for_name()

                # Return name if we know it
                if name:
                    result.dict = name
                else:
                    result.dict = "unknown"

            result.result = "Success"
            return result


    def preempted(self):
        #Procedure in case the call gets cancelled
        pass

    def load_known_faces(self):
        if os.path.exists(self.encodings_file):
            with open(self.encodings_file, "r") as f:
                try:
                    data = json.load(f)
                    rospy.loginfo("Loaded face database successfully.")
                    
                    data["encodings"] = [np.array(encoding, dtype=np.float64) for encoding in data["encodings"]]
                    return data
                except json.JSONDecodeError as e:
                    rospy.logerr(f"Error decoding JSON: {e}")
        else:
            rospy.logwarn(f"Face database file {self.encodings_file} does not exist.")
        return {"encodings": [], "ids": [], "names": [], "locations": []}


    
    def save_known_faces(self):
        """Save known face encodings and names to the database."""
        # Convert numpy arrays to lists for JSON serialization
        data = {
            'encodings': [encoding.tolist() for encoding in self.known_faces["encodings"]],
            'ids': self.known_faces["ids"],
            'names': self.known_faces["names"],
            'locations': self.known_faces["locations"]
        }
        
        try:
            with open(self.encodings_file, "w") as f:
                json.dump(data, f, indent=4)
            rospy.loginfo("Face data saved successfully.")
        except Exception as e:
            rospy.logerr(f"Error saving face data: {e}")

        
    def face_id_callback(self, msg):
        """Subscribe to detected face IDs and listen to their aligned image topics."""
        #if self.enable:
        for face_id in msg.ids:
            if face_id not in self.face_images_subs:
                # Subscribe to the aligned face image instead of landmarks
                aligned_topic = f"/humans/faces/{face_id}/aligned"
                self.face_images_subs[face_id] = rospy.Subscriber(aligned_topic, Image, self.face_image_callback, callback_args=face_id)



    def face_image_callback(self, msg, face_id):
        """Process the received aligned face image. Save it if it's a new one."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        face_encodings = face_recognition.face_encodings(cv_image)
        
        if len(face_encodings) > 0:
            encoding = face_encodings[0]
            match_id = self.find_matching_face(encoding)
            
            if match_id:
                rospy.loginfo(f"Recognized face {face_id} as {match_id}")
                self.current_id = match_id
            else:
                name = None
                new_id = self.save_new_face(face_id, encoding, cv_image, name)
                rospy.loginfo(f"Saved new face {face_id} as {new_id} (Name: {name}).")
                self.current_id = new_id
            
            # Save latest known location if you know the name
            self.add_location_to_face(face_id)
            
        
    def find_matching_face(self, encoding):
        """Find the closest match in the database."""
        if len(self.known_faces["encodings"]) == 0:
            return None
        
        matches = face_recognition.compare_faces(self.known_faces["encodings"], encoding, tolerance = 0.4)
        if True in matches:
            return self.known_faces["ids"][matches.index(True)]
        return None
    
    def save_new_face(self, face_id, encoding, image, name="unknown"):
        """Save new face encoding, image, and name."""
        new_id = f"person_{len(self.known_faces['ids']) + 1}"
        self.known_faces["encodings"].append(encoding)
        self.known_faces["ids"].append(new_id)
        self.known_faces["names"].append(name)
        self.known_faces["locations"].append({}) 
        
        # Save encoding and names to the database
        self.save_known_faces()
        
        # Save the image to disk
        face_path = os.path.join(self.database_path, f"{new_id}.jpg")
        cv2.imwrite(face_path, image)
        
        return new_id


    def add_name_to_face(self, name):
        """Assign a name to the currently seen face ID."""
        face_id = self.current_id
        if face_id:
            try:
                index = self.known_faces["ids"].index(face_id)
                self.known_faces["names"][index] = name
                self.save_known_faces()
                rospy.loginfo(f"Assigned name {name} to face ID {face_id}")
            except ValueError:
                rospy.logwarn("Face ID not found in known faces.")
        else:
            rospy.logwarn("No recognized face to assign a name.")


    def add_location_to_face(self, temporal_face_id):
        """Assign a last seen location in the map to the currently seen face ID."""
        face_id = self.current_id
        if face_id:
            try:
                index = self.known_faces["ids"].index(face_id)
            except ValueError:
                rospy.logwarn("Face ID not found for location update.")
                return

            # Update name
            name = self.known_faces["names"][index]

            # Search body ID
            body_id = None
            print("Temporal:",temporal_face_id)
            for id, person in self.hri_listener.tracked_persons.items():
                rospy.loginfo("Detected personID %s with matching: faceID: %s, bodyID: %s, voiceID: %s",

                          id, person.face_id, person.body_id, person.voice_id)
                if person.face_id == temporal_face_id:
                    print("Match:",person.face_id)
                    print("Body:",person.body_id)
                    body_id = person.body_id
                    rospy.loginfo(f"Found body_id: {body_id}")
                    break
            if body_id: # and name:
                topic_name = f"/humans/bodies/{body_id}/position"
                rospy.Subscriber(topic_name, Point, self.position_callback)

        else:
            rospy.logwarn("No recognized face to assign a location.")


    def position_callback(self, msg):
        """Callback to position of face being currently detected."""
        try:
            index = self.known_faces["ids"].index(self.current_id)
        except ValueError:
            rospy.logwarn("Current face ID not found for position update.")
            return
        self.known_faces["locations"][index] = {
            "x": msg.x,
            "y": msg.y,
            "z": msg.z
        }
        self.save_known_faces()

    
    def search_for_name(self):
        """ Search if the name of the person the robot is looking at currently has a name saved.
            Send it to the brain to greet person by name."""
        face_id = self.current_id
        if face_id:
            try:
                index = self.known_faces["ids"].index(face_id)
            except ValueError:
                rospy.logwarn("Face ID not found for location update.")
                return

        # Obtain name
        name = self.known_faces["names"][index]

        # If a name has been obtained, return it
        if name:
            return name
        
        return





    


if __name__ == '__main__':
    rospy.init_node('face_recognition_node', anonymous=False)    
    server = StatusUpdate(rospy.get_name(), FaceRecognitionNode)
    rospy.spin()
