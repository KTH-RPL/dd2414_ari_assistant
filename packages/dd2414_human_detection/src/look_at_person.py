#!/usr/bin/env python3
import rospy
import pyhri 
from geometry_msgs.msg import Point, PointStamped
from dd2414_status_update import StatusUpdate


class LookAtFace:
    def __init__(self):
        # HRI Listener for face detection
        self.hri = pyhri.HRIListener()

        # Publisher for gaze target
        self.look_at_pub = rospy.Publisher("/look_at", PointStamped, queue_size=10)

        self.active = True
        self.looking_ahead = False

    def action(self,goal):
        if(goal == "start"):
            self.active = True
            return "Success"
        elif(goal == "stop"):
            self.active = False
            return "Success"
        else:
            return "Failure"
    
    def preempted(self):
        self.active = False
        pass

    def trackFaces(self):

        faces = self.hri.faces
        if self.active and len(faces) > 0 and list(faces.values())[-1].valid:  # Check if valid faces are detected

            self.looking_ahead = False

            rospy.loginfo(f"LookAtFace::Found faces {list(faces)}")

            # Select last face
            face = list(faces.values())[-1]

            if(face.valid):
                try:
                    # Get face transform in the head camera frame
                    T = face.transform()
                    trans = T.transform.translation

                    # Create and publish gaze target
                    target = PointStamped(point=Point(x=trans.x, y=trans.y, z=trans.z), header=T.header)
                    self.look_at_pub.publish(target)

                except Exception as e:
                    rospy.logwarn(f"Could not transform face position: {e}")
                

        elif(not self.looking_ahead):
            rospy.loginfo(f"LookAtFace::No faces detected")
            target = PointStamped(point=Point(x=10, y=0, z=0))
            target.header.frame_id = '/sellion_link'

            self.look_at_pub.publish(target)
            self.looking_ahead = True

        return
            

if __name__ == '__main__':
    rospy.init_node("face_gaze_tracker")
    server = StatusUpdate(rospy.get_name(),LookAtFace)

    while not rospy.is_shutdown():
        server.node.trackFaces()
        rospy.Rate(1).sleep()

    
