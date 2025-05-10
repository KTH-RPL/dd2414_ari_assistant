#!/usr/bin/env python3

import rospy
import math
import actionlib
import numpy as np
import dd2414_brain_v2.msg as brain

from tf import TransformListener
from tf import transformations as t
from pyhri import HRIListener
from control_msgs.msg import FollowJointTrajectoryActionGoal, PointHeadActionGoal
from geometry_msgs.msg import PointStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dd2414_status_update import StatusUpdate

class ARIHeadFollower:
    def __init__(self):
        self.hri_listener = HRIListener()

        self._tf_Listener = TransformListener()


        # Publisher for head movement
        self.look_at_goal = rospy.Publisher("/head_controller/point_head_action/goal",PointHeadActionGoal,queue_size=1)

        self.move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def action (self,goal):
        return self.track_user()

    def track_user(self):
        
        result = brain.BrainResult()

        if not self.stop: #If its running or just started running
            if len(self.hri_listener.bodies) > 0 : #Are there any bodies?

                bodies = list(self.hri_listener.bodies.values())[0]

                #Get the transform from ARI base_link to body
                (t_bodies,r_bodies) = self._tf_Listener.lookupTransform("base_link",bodies.frame,rospy.Time(0))


                # Convert position to joint angles
                if t_bodies[0] <= 1.5: #If the distanced traveled on X on the transform after the Rotation has been applied

                    #Turn Head towards the body
                    angle = self.turn_head(t_bodies)

                    if angle > 1.4: #If angle is greater turn to one side
                        facing = t.quaternion_from_euler(0,0,np.pi/2)
                        result = self.nav_move_base(0,0,0,facing[0],facing[1],facing[2],facing[3])
                    elif angle < -1.4: #Else if the angle is lesser turn to the other side 
                        facing = t.quaternion_from_euler(0,0,-np.pi/2)
                        result = self.nav_move_base(0,0,0,facing[0],facing[1],facing[2],facing[3])
                    #If not dont move the body
                else:
                    self.approach(t_bodies)


            result.result = "Working"
        else:
            result.result = "Success"
        
        return result
    
    def turn_head(self,t_bodies):
        #Construct Goal to point face to
        head_goal = PointHeadActionGoal()
        head_goal.goal.target.header.frame_id = "base_link"
        head_goal.goal.pointing_axis.x = 0.0
        head_goal.goal.pointing_frame = "sellion_link"

        angle = math.atan2(point.y,point.x)

        if abs(angle) > 1.4: #If the angle of turning is greater than face forward and it will turn the body
            point = Point(x=1.0, y=-0.0, z=1.5)
        else: #If not just look at the user
            point = Point(x=t_bodies[0], y=t_bodies[1], z=t_bodies[2])
        
        head_goal.goal.target.point = point
        self.look_at_goal.publish(head_goal)
        return angle

    def nav_move_base(self,x,y,z,rx,ry,rz,rw):
        self.move_client.wait_for_server(self.timeout)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.x = rx
        goal.target_pose.pose.orientation.y = ry
        goal.target_pose.pose.orientation.z = rz
        goal.target_pose.pose.orientation.w = rw

        rospy.logdebug("[FOLLOW_USER    ]:Sending goal")
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()

        rospy.logdebug(f"[FOLLOW_USER    ]:Goal state:{self.move_client.get_state()}")
        rospy.logdebug("[FOLLOW_USER    ]:"+self.move_client.get_goal_status_text())
        
        return self.move_client.get_state()

    def approach(self,t_bodies):
        #Position of User
        x_user = t_bodies[0]
        y_user = t_bodies[1]

        #
        radius = math.sqrt(x_user**2 + y_user**2) - 1.0
        theta_user = math.atan2(y_user,x_user)

        x_goal = radius * math.cos(theta_user)
        y_goal = radius * math.sin(theta_user)

if __name__ == "__main__":
    rospy.init_node("follow_user",log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(),ARIHeadFollower)