#!/usr/bin/env python3

import rospy
from tf import transformations as t
import numpy as np
import pyhri
from hri_msgs.msg import IdsList
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

class BodyOrientationListener:

    def __init__(self,

                base_frame = "head_front_camera_link",

                 threshold=30):

        self.hri_listener = pyhri.HRIListener()
        self.base_frame = base_frame
        self.threshold = threshold

        self.id_pub = rospy.Publisher('person_looking_at_robot', String, queue_size=10)

        self.rate = rospy.Rate(10)
        self.is_body_facing_robot = False


    def run(self):

        """ The run function implement the main

            functionality of the BodyOrientationListener

            object, that is understanding which bodies

            are oriented toward the robot. Here, the

            base_frame specified during the object

            initialisation represent the robot. """

        while not rospy.is_shutdown():

            bodies = self.hri_listener.bodies.items()
            bodies_facing_robot = []

            for body in bodies:

                transform = body[1].transform(self.base_frame)
                trans = transform.transform.translation
                rot = transform.transform.rotation

                translation_matrix = t.translation_matrix((trans.x, trans.y, trans.z))
                quaternion_matrix = t.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))
                transform = t.concatenate_matrices(translation_matrix, quaternion_matrix)
                inv_transform = t.inverse_matrix(transform)
                b2r_translation_x = inv_transform[0, 3]
                b2r_translation_y = inv_transform[1, 3]
                b2r_xy_norm = np.linalg.norm([b2r_translation_x, b2r_translation_y], ord = 2)
                
                # Does the base frame lie inside the body frame-based cone of attention?

                if np.arccos(b2r_translation_x/b2r_xy_norm) < (self.threshold/180*np.pi) and b2r_translation_x > 0:

                    bodies_facing_robot.append(body[0])
                    print("namespace: ", body[1].ns)
                    print("body: ", body[1])
                    self.id_pub.publish((body[0]))
            
            if(len(bodies_facing_robot) == 0):
                self.id_pub.publish("")


            self.rate.sleep()


if __name__=="__main__":


    rospy.init_node("body_orientation_listener")
    bol = BodyOrientationListener(threshold=20)
    bol.run()