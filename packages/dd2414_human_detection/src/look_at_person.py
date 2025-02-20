#!/usr/bin/env python3
import rospy
import pyhri 
from geometry_msgs.msg import Point, PointStamped

# Initialize ROS node
rospy.init_node("face_gaze_tracker")

# HRI Listener for face detection
hri = pyhri.HRIListener()

# Publisher for gaze target
look_at_pub = rospy.Publisher("/look_at", PointStamped, queue_size=1)

r = rospy.Rate(10)

while not rospy.is_shutdown():
    if len(hri.faces) > 0:  # Check if faces are detected
        face = list(hri.faces.values())[0]  # Select the first detected face
        print(f"Looking at {face.id}")

        try:
            # Get face transform in the head camera frame
            T = face.transform()
            trans = T.transform.translation
            
            # Create and publish gaze target
            target = PointStamped(point=Point(x=trans.x, y=trans.y, z=trans.z), header=T.header)
            look_at_pub.publish(target)
        except Exception as e:
            rospy.logwarn(f"Could not transform face position: {e}")
    else:
        print("No faces detected")
    
    r.sleep()