#!/usr/bin/env python3
import rospy
import pyhri 
from geometry_msgs.msg import Point, PointStamped
from dd2414_status_update import StatusUpdate
import dd2414_brain_v2.msg as brain 


class LookAtFace:
    def __init__(self):

        self.result = brain.BrainResult()

        # HRI Listener for face detection
        self.hri = pyhri.HRIListener()

        # Publisher for gaze target
        self.look_at_pub = rospy.Publisher("/look_at", PointStamped, queue_size=1)

        self.active = True
        self.looking_ahead = False
        self.tick = 0

        rospy.loginfo("[LOOK_AT_PERSON ]:Initialized")
        self.string_header = "[LOOK_AT_PERSON ]:"

    # When prompted, either start or stop looking at faces
    def action(self,goal):
        if(goal.goal == "start"):
            self.active = True
            self.result.result = "Success"
            rospy.logwarn(f"{self.string_header} Starting look at face")

        elif(goal.goal == "stop"):
            self.active = False
            self.look_forward()
            self.result.result = "Success"
            rospy.logwarn(f"{self.string_header} Stopping look at face")
        
        else:
            rospy.logwarn(f"{self.string_header} Invalid goal")
            self.result.result = "Failure"
        
        return self.result
    
    def preempted(self):
        self.active = False
        pass

    def trackFaces(self):

        self.tick += 1

        faces = self.hri.faces

        # Check if valid faces are detected
        if self.active and len(faces) > 0 and list(faces.values())[-1].valid:

            self.tick = 0
            self.looking_ahead = False

            rospy.logdebug(f"[LOOK_AT_PERSON ]:Found faces {list(faces)}")

            # Select last face
            face = list(faces.values())[-1]

            if(face.valid):
                try:
                    # Get face transform in the head camera frame
                    T = face.transform()
                    trans = T.transform.translation

                    # Create and publish gaze target
                    target = PointStamped(point=Point(x=trans.x, y=trans.y, z=trans.z), header=T.header)
                    if(self.active):
                        self.look_at_pub.publish(target)

                except Exception as e:
                    rospy.logwarn(f"[LOOK_AT_PERSON ]:Could not transform face position: {e}")

        # If no face has been detected for 2s, look forward 
        elif(self.tick > 20 and self.active):
            rospy.logdebug("[LOOK_AT_PERSON ]:No faces detected for 2s, looking forward")
            self.look_forward()
            self.tick = 0

        # As soon as the face disappears out of view, publish a new goal (continue looking 
        # the same direction), to prevent gaze-manager from crashing
        elif(not self.looking_ahead):
            if(self.active):
                rospy.logdebug("[LOOK_AT_PERSON ]:No faces detected")
                target = PointStamped(point=Point(x=10, y=0, z=0))
                target.header.frame_id = '/sellion_link'

                self.look_at_pub.publish(target)
                self.looking_ahead = True
            else:
                self.look_forward()

        return
    
    def look_forward(self):
        target = PointStamped(point=Point(x=50, y=0, z=0))
        target.header.frame_id = '/base_link'

        self.look_at_pub.publish(target)

if __name__ == '__main__':
    rospy.init_node("face_gaze_tracker", log_level=rospy.WARN)
    server = StatusUpdate(rospy.get_name(), LookAtFace)

    while not rospy.is_shutdown():
        server.node.trackFaces()
        rospy.Rate(10).sleep()

    
