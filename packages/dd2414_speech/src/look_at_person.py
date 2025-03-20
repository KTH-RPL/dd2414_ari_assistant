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

tracking = True
while not rospy.is_shutdown():
    if len(hri.faces) > 0 and list(hri.faces.values())[-1].valid:  # Check if valid faces are detected
        
        print((list(hri.faces)))

        # Select last face
        face = list(hri.faces.values())[-1]
        if(face.valid):
            try:
                # Get face transform in the head camera frame
                T = face.transform()
                trans = T.transform.translation
                
                # Create and publish gaze target
                target = PointStamped(point=Point(x=trans.x, y=trans.y, z=trans.z), header=T.header)

            except Exception as e:
                rospy.logwarn(f"Could not transform face position: {e}")    
    else:
        target = PointStamped(point=Point(x=10, y=0, z=0))
        target.header.frame_id = '/sellion_link'

    look_at_pub.publish(target)
        
    r.sleep()
