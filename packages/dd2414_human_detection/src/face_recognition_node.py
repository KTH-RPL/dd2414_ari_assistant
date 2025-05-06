#!/usr/bin/env python3
import rospy
import os
import cv2
import face_recognition
from tf import TransformListener
from sensor_msgs.msg import Image
from hri_msgs.msg import IdsList
from pyhri import HRIListener
from cv_bridge import CvBridge
import json
import numpy as np
from dd2414_status_update import StatusUpdate
import dd2414_brain_v2.msg as brain
from geometry_msgs.msg import Point
from pal_zoi_detector.srv import GetPointZoI, GetPointZoIRequest
import time

class FaceRecognitionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.hri_listener = HRIListener()

        self.face_encoding_buffer = {}  # Store multiple encodings per face
        self.encoding_threshold = 0.6   # Similarity threshold
        self.required_encodings = 5     # Number of encodings needed for stability

        self.target_name = None
        self.current_id = None

        self._tf_Listener = TransformListener()
        
        
        # Paths for saving data
        self.database_path = os.path.dirname(__file__)
        self.encodings_file = os.path.join(self.database_path, "face_database.json")
        os.makedirs(self.database_path, exist_ok=True)
        
        # Load known faces from the file
        self.known_faces = self.load_known_faces()
        
        # ROS subscribers
        self.face_ids_sub = rospy.Subscriber("/humans/faces/tracked", IdsList, self.face_id_callback)
        self.face_images_subs = {}
        rospy.loginfo("[FACERECOGNITION]:Initialized")
        self.string_header = "[FACERECOGNITION]:"

        self.working_count = 0
        

    def action(self,goal):
        """
        This function will execute when it receives a goal from the brain. It will contain a string (goal.goal)
        which indicates the name of the person we are currently seeing. 
        """
        result = brain.BrainResult()

        if goal.goal:
            self.target_name = goal.goal

            # If face not yet seen FAILURE, can not save name or search for name
            if self.current_id is None:
                rospy.logwarn("[FACERECOGNITION]:No face detected to save.")
                result.result = "Failed"
                return result

            # Save name if its not unknown
            if self.target_name != "unknown":
                rospy.loginfo(f"[FACERECOGNITION]:Save name: {self.target_name}")
                self.add_name_to_face(self.target_name)

                # Return name that was saved
                result.in_dic = json.dumps({"name" : self.target_name})

            else:
                rospy.logdebug(f"[FACERECOGNITION]:Name unknown. Searching if we already know it.")
                name = self.search_for_name()

                # Return name if we know it
                if name:
                    result.in_dic = json.dumps({"name" : name })
                else:
                    result.in_dic = json.dumps({"name" : "unknown" })

            result.result = "Success"
            rospy.logdebug(result)
        else:
            rospy.logwarn("[FACERECOGNITION]:No goal provided.")
            result.result = "Failure"
        
        return result

    def preempted(self):
        # Procedure in case the call gets cancelled
        pass

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
        return {"encodings": {}, "ids": [], "names": [], "coordinates": [], "room": [] }


    
    def save_known_faces(self):
        """Save known face encodings and names to the database."""
        # Convert numpy arrays to lists for JSON serialization
        data = {
            'encodings': {key: [encoding.tolist() for encoding in encodings] 
                      for key, encodings in self.known_faces["encodings"].items()},
            'ids': self.known_faces["ids"],
            'names': self.known_faces["names"],
            'coordinates': self.known_faces["coordinates"],
            'room': self.known_faces["room"]
        }
        
        try:
            with open(self.encodings_file, "w") as f:
                json.dump(data, f, indent=4)
            rospy.logdebug("[FACERECOGNITION]:Face data saved successfully.")
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
        match_id = None
        
        if len(face_encodings) > 0:
            encoding = face_encodings[0]

            # Store encoding in buffer
            if face_id not in self.face_encoding_buffer:
                self.face_encoding_buffer[face_id] = []
            
            self.face_encoding_buffer[face_id].append(encoding)

            # Keep only the last 'required_encodings' values
            if len(self.face_encoding_buffer[face_id]) > self.required_encodings:
                self.face_encoding_buffer[face_id].pop(0)

            # If we have enough encodings, check if they are stable
            if len(self.face_encoding_buffer[face_id]) >= self.required_encodings:
                if self.is_stable_encoding(face_id):
                    match_id = self.find_matching_face(encoding)
                
                                
                if match_id:
                    rospy.logdebug(f"[FACERECOGNITION]:Recognized face {face_id} as {match_id}")
                    self.current_id = match_id
                else:
                    name = None
                    new_id = self.save_new_face(face_id, encoding, cv_image, name)
                    rospy.loginfo(f"[FACERECOGNITION]:Saved new face {face_id} as {new_id} (Name: {name}).")
                    self.current_id = new_id
                
                # Save latest known location if you know the name
                self.add_location_to_face(face_id)
                del self.face_encoding_buffer[face_id]  

        
    def is_stable_encoding(self, face_id):
        """Check if stored encodings match using face_recognition.compare_faces."""
        encodings = self.face_encoding_buffer[face_id]
        first_encoding = encodings[0]  # Use the first stored encoding as reference

        matches = face_recognition.compare_faces(encodings, first_encoding, tolerance=self.encoding_threshold)
        match_ratio = sum(matches) / len(matches)  # Calculate percentage of matches

        return match_ratio >= 0.8  # Require 80% of encodings to match

            
        
    def find_matching_face(self, encoding):
        """Find the closest match in the database."""
        if len(self.known_faces["encodings"]) == 0:
            return None
        
        for person_id, person_encodings in self.known_faces["encodings"].items():
            matches = face_recognition.compare_faces(person_encodings, encoding, tolerance=0.6)
            match_ratio = sum(matches) / len(matches)  # Calculate percentage of matches
            
            if match_ratio >= 0.8:  # Require 80% of encodings to match
                return person_id
    
    def save_new_face(self, face_id, encoding, image, name="unknown"):
        """Save new face encoding, image, and name."""
        new_id = f"person_{len(self.known_faces['ids']) + 1}"

        # Store multiple encodings for the new person
        if new_id not in self.face_encoding_buffer:
            self.face_encoding_buffer[new_id] = []
        
        self.face_encoding_buffer[new_id].append(encoding)

        # Only store 5 encodings for each person
        if len(self.face_encoding_buffer[new_id]) > self.required_encodings:
            self.face_encoding_buffer[new_id].pop(0)

        # After collecting enough encodings, save to known faces
        if len(self.face_encoding_buffer[new_id]) >= self.required_encodings:
            self.known_faces["encodings"][new_id] = self.face_encoding_buffer[new_id]
            self.known_faces["ids"].append(new_id)
            self.known_faces["names"].append(name)
            self.known_faces["coordinates"].append({})
            self.known_faces["room"].append(None)

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
                rospy.logdebug(f"Assigned name {name} to face ID {face_id}")
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

            faces = list(self.hri_listener.faces.values())

            for face in faces:
                    
                transform_face = face.transform()

                (t_face,r_face) = self._tf_Listener.lookupTransform("map",face.frame,rospy.Time(0))

                x = round(t_face[0], 1)
                y = round(t_face[1], 1)
                z = 0

                self.known_faces["coordinates"][index] = {
                    "x": x,  
                    "y": y
                }

                # Save the room that corresponds to that location
                room = self.get_zoi_for_point(x, y, z)
                
                self.known_faces["room"][index] = room

                self.save_known_faces()

        else:
            rospy.logwarn("No recognized face to assign a location.")


    
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
    
    def get_zoi_for_point(self, x, y, z):
        """
        Calls the /get_zoi service, passing a point (x, y, z),
        and returns the resulting Zone of Interest (ZoI).
        """
        # Wait for the service to be available (with timeout)
        try:
            rospy.wait_for_service('/get_zoi', timeout=0.5)  # Wait half a second
        except rospy.ROSException as e:
            rospy.logerr(f"[FACERECOGNITION]:Timeout waiting for service /get_zoi: {e}")
            return None

        try:
            # Create a service proxy for the GetPointZoI service
            get_zoi = rospy.ServiceProxy('/get_zoi', GetPointZoI)

            # Create the Point message with the coordinates (x, y, z)
            point = Point(x, y, z)

            # Call the service with the point and get the response
            response = get_zoi(point)

            # Access the first zone of interest (zois is a list)
            if response.zois.zois:
                zone_of_interest = response.zois.zois[0]  # Get the first ZoI string
                rospy.loginfo(f"Zone of Interest: {zone_of_interest}")
                return zone_of_interest
            else:
                rospy.logwarn("No zones of interest returned.")
                return None

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None





    


if __name__ == '__main__':
    rospy.init_node('face_recognition_node', anonymous=False,log_level=rospy.INFO)    
    server = StatusUpdate(rospy.get_name(), FaceRecognitionNode)
    rospy.spin()
