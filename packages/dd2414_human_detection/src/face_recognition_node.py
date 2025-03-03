#!/usr/bin/env python3
import rospy
import os
import cv2
import face_recognition
from sensor_msgs.msg import Image
from hri_msgs.msg import IdsList
from cv_bridge import CvBridge
import json
import numpy as np

class FaceRecognitionNode:
    def __init__(self):
        rospy.init_node('face_recognition_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Paths for saving data
        self.database_path = os.path.expanduser("~/catkin_ws/src/dd2414/dd2414_human_detection/src")
        self.encodings_file = os.path.join(self.database_path, "face_database.json")
        os.makedirs(self.database_path, exist_ok=True)
        
        # Load known faces from the file
        self.known_face_encodings, self.known_face_ids, self.known_face_names = self.load_known_faces()
        
        # ROS subscribers
        self.face_ids_sub = rospy.Subscriber("/humans/faces/tracked", IdsList, self.face_id_callback)
        self.face_images_subs = {}
        self.detected_faces = {}

    def load_known_faces(self):
        """Load known face encodings and associated names from the database."""
        if os.path.exists(self.encodings_file):
            with open(self.encodings_file, "r") as f:
                try:
                    data = json.load(f)
                    rospy.loginfo("Loaded face database successfully.")
                    return data['encodings'], data['ids'], data['names']
                except json.JSONDecodeError as e:
                    rospy.logerr(f"Error decoding JSON: {e}")
        else:
            rospy.logwarn(f"Face database file {self.encodings_file} does not exist.")
        return [], [], []


    def save_known_faces(self):
        """Save known face encodings and names to the database."""
        # Convert numpy arrays to lists for JSON serialization
        data = {
            'encodings': [encoding.tolist() for encoding in self.known_face_encodings],  # Convert ndarray to list
            'ids': self.known_face_ids,
            'names': self.known_face_names
        }

        try:
            with open(self.encodings_file, "w") as f:
                json.dump(data, f, indent=4)  # Pretty-printing JSON for better readability
            rospy.loginfo("Face data saved successfully.")
        except Exception as e:
            rospy.logerr(f"Error saving face data: {e}")

        
    def face_id_callback(self, msg):
        """Subscribe to detected face IDs and listen to their aligned image topics."""
        for face_id in msg.ids:
            if face_id not in self.face_images_subs:
                # Subscribe to the aligned face image instead of landmarks
                aligned_topic = f"/humans/faces/{face_id}/aligned"
                self.face_images_subs[face_id] = rospy.Subscriber(aligned_topic, Image, self.face_image_callback, callback_args=face_id)


    def face_image_callback(self, msg, face_id):
        """Process the received aligned face image and associate a name if provided."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        face_encodings = face_recognition.face_encodings(cv_image)
        
        if len(face_encodings) > 0:
            encoding = face_encodings[0]
            match_id = self.find_matching_face(encoding)
            
            if match_id:
                rospy.loginfo(f"Recognized face {face_id} as {match_id}")
            else:
                # Ask for a name if it's available
                # name = self.get_face_name(face_id)
                name = None
                new_id = self.save_new_face(face_id, encoding, cv_image, name)
                rospy.loginfo(f"Saved new face {face_id} as {new_id} (Name: {name})")
        
    def find_matching_face(self, encoding):
        """Find the closest match in the database."""
        if len(self.known_face_encodings) == 0:
            return None
        
        matches = face_recognition.compare_faces(self.known_face_encodings, encoding)
        if True in matches:
            return self.known_face_ids[matches.index(True)]
        return None
    
    def save_new_face(self, face_id, encoding, image, name="Unknown"):
        """Save new face encoding, image, and name."""
        new_id = f"person_{len(self.known_face_ids) + 1}"
        self.known_face_encodings.append(encoding)
        self.known_face_ids.append(new_id)
        self.known_face_names.append(name)
        
        # Save encoding and names to the database
        self.save_known_faces()
        
        # Save the image to disk
        face_path = os.path.join(self.database_path, f"{new_id}.jpg")
        cv2.imwrite(face_path, image)
        
        return new_id
    
    def get_face_name(self, face_id):
        """Simulate getting the name for the face if provided."""
        # In real scenarios, this could be done by querying an external source or user input
        # Here we use a simple logic for testing
        name = input(f"Enter name for face {face_id} (or press Enter to skip): ")
        return name if name else "Unknown"

if __name__ == '__main__':
    node = FaceRecognitionNode()
    rospy.spin()
