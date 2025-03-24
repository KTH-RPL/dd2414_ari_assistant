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
        self.look_at_pub = rospy.Publisher("/look_at", PointStamped, queue_size=1)

        self.active = True
        self.looking_ahead = False
        self.tick = 0

    def action(self,goal):
        if(goal == "start"):
            self.active = True
            return "Success"
        elif(goal == "stop"):
            self.active = False
            self.look_forward()
            return "Success"
        else:
            return "Failure"
    
    def preempted(self):
        self.active = False
        pass

    def trackFaces(self):

        self.tick += 1

        faces = self.hri.faces
        if self.active and len(faces) > 0 and list(faces.values())[-1].valid:  # Check if valid faces are detected

            self.tick = 0

            self.looking_ahead = False

            rospy.loginfo(f"LookAtFace::Found faces {list(faces)}")

            # Select last face
            face = list(faces.values())[-1]

            print('test1')

            if(face.valid):
                print('test2')
                try:
                    print('test3')
                    # Get face transform in the head camera frame
                    T = face.transform()
                    trans = T.transform.translation

                    print('test4')

                    # Create and publish gaze target
                    target = PointStamped(point=Point(x=trans.x, y=trans.y, z=trans.z), header=T.header)
                    self.look_at_pub.publish(target)

                    print('test5')

                except Exception as e:
                    rospy.logwarn(f"Could not transform face position: {e}")
                
        elif(self.tick > 20):
            rospy.loginfo(f"LookAtFace::No faces detected for 2s, looking forward")
            self.look_forward()
            self.tick = 0

        elif(not self.looking_ahead):
            rospy.loginfo(f"LookAtFace::No faces detected")
            target = PointStamped(point=Point(x=10, y=0, z=0))
            target.header.frame_id = '/sellion_link'

            self.look_at_pub.publish(target)
            self.looking_ahead = True

        return
    
    def look_forward(self):
        target = PointStamped(point=Point(x=50, y=0, z=0))
        target.header.frame_id = '/base_link'

        self.look_at_pub.publish(target)



if __name__ == '__main__':
    rospy.init_node("face_gaze_tracker")
    server = StatusUpdate(rospy.get_name(),LookAtFace)

    while not rospy.is_shutdown():
        server.node.trackFaces()
        rospy.Rate(10).sleep()

    
