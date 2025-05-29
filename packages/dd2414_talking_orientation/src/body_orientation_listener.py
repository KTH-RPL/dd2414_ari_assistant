#!/usr/bin/env python3
import actionlib.msg
import rospy
import numpy as np
import pyhri
import sys
import actionlib
import move_base_msgs.msg
import dd2414_brain_v2.msg as brain

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

from tf import transformations as t
from tf import TransformBroadcaster as tf_B
from tf import TransformListener
from tf import LookupException,ExtrapolationException,ConnectivityException
from tf2_ros import Buffer
from hri_msgs.msg import IdsList, LiveSpeech
from geometry_msgs.msg import Point, PointStamped
from dd2414_status_update import StatusUpdate
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from std_msgs.msg import String
from ollama import Client


class BodyOrientationListener:

    def __init__(self, base_frame = "head_front_camera_link", threshold=20):
        self.hri_listener = pyhri.HRIListener()
        self.base_frame = base_frame
        self.threshold = threshold
        self.rate = rospy.Rate(1)
        self.br = tf_B()
        self._tf_buffer = Buffer()
        self._tf_Listener = TransformListener()
        rospy.loginfo("[BODYORIENTATION]:Initialized")
        self.string_header = "[BODYORIENTATION]:"

    def quaternion_euler(self,quaternion):
        euler = t.euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])
        return euler[2]

    def nav_move_base(self,x,y,z,rx,ry,rz,rw):
        move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        move_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.x = rx
        goal.target_pose.pose.orientation.y = ry
        goal.target_pose.pose.orientation.z = rz
        goal.target_pose.pose.orientation.w = rw

        move_client.send_goal(goal)
        move_client.wait_for_result()

        rospy.logdebug(f"[BODYORIENTATION]:Goal state:{move_client.get_state()}")
        rospy.logdebug("[BODYORIENTATION]:"+move_client.get_goal_status_text())
        
        return move_client.get_state()
    
    def action(self,goal):
        #Procedure when the action gets called
        #THE POSSIBLE RETURN VALUES ARE "Success","Failure","Working"
        return self.run()

    def preempted(self):
        #Procedure in case the call gets cancelled
        pass

    def normalize_quaternion(self, q):
        norm = np.linalg.norm(q)
        if norm == 0:
            rospy.logwarn(f"{self.string_header} Quaternion has zero length, using default identity quaternion.")
            return [0.0, 0.0, 0.0, 1.0]  # Identity quaternion
        return [x / norm for x in q]


    def run(self):
        result_brain = brain.BrainResult()
        while not rospy.is_shutdown():
            bodies = self.hri_listener.bodies.items()
            bodies_facing_robot = []
            
            for body in bodies:
                print("body_" + body[0])
                right_euler = t.quaternion_from_euler(0,0,np.pi/3)
                left_euler  = t.quaternion_from_euler(0,0,-np.pi/3)

                self.br.sendTransform((0.5,0.8,0.0),
                                 (left_euler[0],left_euler[1],left_euler[2],left_euler[3]),
                                 rospy.Time.now(),
                                 "left_" + body[0],
                                 "body_" + body[0])
                
                self.br.sendTransform((0.5,-0.8,0.0),
                                 (right_euler[0],right_euler[1],right_euler[2],right_euler[3]),
                                 rospy.Time.now(),
                                 "right_" + body[0],
                                 "body_" + body[0])
                
                try:
                    (left_trans, left_rot) = self._tf_Listener.lookupTransform("map","left_"+body[0],rospy.Time(0))
                    left_rot = self.normalize_quaternion(left_rot) # Normalize
                    
                    (right_trans, right_rot) = self._tf_Listener.lookupTransform("map","right_"+body[0],rospy.Time(0))
                    right_rot = self.normalize_quaternion(right_rot)

                    transform_valid = True
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    rospy.logdebug(e)
                    rospy.logdebug(f"{self.string_header} Transform not published yet")
                    transform_valid = False
                    continue

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

                if np.arccos(b2r_translation_x/b2r_xy_norm) < (self.threshold/180*np.pi) and b2r_translation_x > 0:
                    rospy.logdebug("[BODYORIENTATION]:Starting movement")
                    bodies_facing_robot.append(body[0])

                    try:
                        if transform_valid:
                            
                            result = self.nav_move_base(left_trans[0],left_trans[1],0,left_rot[0],left_rot[1],left_rot[2],left_rot[3])

                            if result == actionlib.GoalStatus.SUCCEEDED:
                                rospy.logdebug(f"{self.string_header} Arrived at target!")
                                result_brain.result = "Success"
                                return result_brain
                                
                            else:
                                rospy.logdebug(f"{self.string_header} Trying right position!")
                                result = self.nav_move_base(right_trans[0],right_trans[1],0,right_rot[0],right_rot[1],right_rot[2],right_rot[3])
                                if result == actionlib.GoalStatus.SUCCEEDED:
                                    result_brain.result = "Success"
                                    return result_brain
                                else:
                                    result_brain.result = "Failure"
                                    return result_brain
                        else:
                            rospy.logdebug(f"{self.string_header} Transform not available!")
                            result_brain.result = "Working"
                            return result_brain
                        
                    except rospy.ROSInterruptException as e:
                        rospy.loginfo(e)
                        result_brain.result = "Working"
                        return result_brain
                        

            self.rate.sleep()


if __name__=="__main__":
    rospy.init_node("body_orientation_listener",log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(),BodyOrientationListener)