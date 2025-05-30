#!/usr/bin/env python3

import rospy
import math
import pyhri
import actionlib
import numpy as np
import dd2414_brain.msg as brain

from tf import TransformListener
from tf import transformations as t
from tf import TransformBroadcaster as tf_B
from tf import LookupException,ExtrapolationException,ConnectivityException
from pyhri import HRIListener
from hri_msgs.msg import LiveSpeech
from control_msgs.msg import FollowJointTrajectoryActionGoal, PointHeadActionGoal
from geometry_msgs.msg import PointStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dd2414_status_update import StatusUpdate

class ARIHeadFollower:
    def __init__(self):
        self.hri_listener = HRIListener()
        self.stop = False
        self.br    = tf_B()
        self._tf_Listener = TransformListener()
        self.string_header = "[FOLLOW_USER    ]:"


        # Publisher for head movement
        #self.look_at_pub  = rospy.Publisher("/look_at",PointStamped,queue_size=1)
        self.look_at_ac = actionlib.SimpleActionClient("/face_gaze_tracker",brain.BrainAction)
        self.look_at_goal = rospy.Publisher("/head_controller/point_head_action/goal",PointHeadActionGoal,queue_size=1)

        self.asr_sub      = rospy.Subscriber('/humans/voices/anonymous_speaker/speech',LiveSpeech, self.asr_result)

        self.timeout = rospy.Duration(10)

        self.move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        rospy.loginfo(f"{self.string_header} Initialized")

    def asr_result(self,msg):
        sentence = msg.final
        rospy.logdebug(f"{self.string_header} Understood sentence: {sentence}")
        
        if sentence.lower() == "stop":
            self.stop = True
            rospy.loginfo(f"{self.string_header}Stoping Following behavior")
            
            # Cancel moving goals
            self.move_client.cancel_all_goals()
            
    def action (self,goal):
        return self.track_user()

    def preempted(self):
        self.stop = False
        self.move_client.cancel_all_goals()
        rospy.loginfo(f"{self.string_header}Follow User Preempted returning None it Fails")
        return

    def track_user(self):
        
        result = brain.BrainResult()

        if not self.stop: #If its running or just started running
            if len(self.hri_listener.bodies) > 0 : #Are there any bodies?

                bodies = list(self.hri_listener.bodies.values())[0]
                bodies.transform()
                #Get the transform from ARI base_link to body
                (t_bodies,r_bodies) = self._tf_Listener.lookupTransform("base_link",bodies.frame,rospy.Time(0))


                T = bodies.transform()
                trans = T.transform.translation

                angle=math.atan2(trans.y,trans.x)*360/(2*math.pi) 
                # Convert position to joint angles
                if t_bodies[0] <= 1.5: #If the distanced traveled on X on the transform after the Rotation has been applied
                    rospy.loginfo(f"{self.string_header}Close to User, Moving Head Only Distance: {t_bodies[0]} Angle: {angle}")
                    #Turn Head towards the body
                    #angle = self.turn_head(t_bodies)
                    angle = self.turn_head_alt(bodies)
                    if angle > 1.4: #If angle is greater turn to one side
                        rospy.loginfo(f"{self.string_header}Close to User, Turning to the Left Distance: {t_bodies[0]} Angle: {angle}")
                        facing = t.quaternion_from_euler(0,0,np.pi/2)
                        self.nav_move_base(0,0,0,facing[0],facing[1],facing[2],facing[3])
                    elif angle < -1.4: #Else if the angle is lesser turn to the other side 
                        rospy.loginfo(f"{self.string_header}Close to User, Turning to the Right Distance: {t_bodies[0]} Angle: {angle}")
                        facing = t.quaternion_from_euler(0,0,-np.pi/2)
                        self.nav_move_base(0,0,0,facing[0],facing[1],facing[2],facing[3])
                    #If not dont move the body
                else:
                    rospy.loginfo(f"{self.string_header}Far from user, approaching Distance: {t_bodies[0]}")
                    #angle = self.turn_head_front()
                    self.turn_head_front_alt()
                    self.approach(t_bodies)


            result.result = "Working"
        else:
            self.stop = False
            result.result = "Success"
        
        return result
    
    def turn_head_alt(self,body):
        # Get face transform in the head camera frame
        T = body.transform()
        trans = T.transform.translation

        angle=math.atan2(trans.y,trans.x) 
        # Create and publish gaze target
#        if abs(angle) > 1.4:
#            if angle > 0: #Positive
#                target=PointStamped(point=Point(x=math.cos(1.4/2),y=math.sin(1.4/2),z=trans.z),header=T.header)
#            else:
#                target=PointStamped(point=Point(x=math.cos(-1.4/2),y=math.sin(-1.4/2),z=trans.z),header=T.header)
#        else:
#            target = PointStamped(point=Point(x=trans.x, y=trans.y, z=trans.z), header=T.header)
        goal = brain.BrainGoal()
        goal.goal = "start"
        self.look_at_ac.send_goal(goal)
#        self.look_at_pub.publish(target)
        return angle 

    def turn_head_front_alt(self):
        #target = PointStamped(point=Point(x=1.0, y=0.0, z=1.5))
        goal = brain.BrainGoal()
        goal.goal = "stop"
        self.look_at_ac.send_goal(goal)
        #self.look_at_pub.publish(target)

#    def turn_head_front(self):
#        head_goal = PointHeadActionGoal()
#        head_goal.goal.target.header.frame_id = "base_link"
#        point = Point(x=1.0, y=-0.0, z=1.5)
#        head_goal.goal.pointing_axis.x = 0.0
#        head_goal.goal.pointing_frame = "sellion_link"
#        head_goal.goal.target.point = point
#        self.look_at_goal.publish(head_goal)

#    def turn_head(self,t_bodies):
#        #Construct Goal to point face to
#        head_goal = PointHeadActionGoal()
#        head_goal.goal.target.header.frame_id = "base_link"
#        point = Point(x=t_bodies[0], y=t_bodies[1], z=t_bodies[2])
#        head_goal.goal.pointing_axis.x = 0.0
#        head_goal.goal.pointing_frame = "sellion_link"
#
#        angle = math.atan2(point.y,point.x)
#
#        if abs(angle) > 1.4: #If the angle of turning is greater than face forward and it will turn the body
#            point = Point(x=1.0, y=-0.0, z=1.5)
#        else: #If not just look at the user
#            point = Point(x=t_bodies[0], y=t_bodies[1], z=t_bodies[2])
#        
#        head_goal.goal.target.point = point
#        self.look_at_goal.publish(head_goal)
#        return angle

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

        rospy.logdebug(f"{self.string_header}Sending goal")
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result(timeout=rospy.Duration(15))

        rospy.logdebug(f"{self.string_header}Goal state:{self.move_client.get_state()}")
        rospy.logdebug(self.string_header+self.move_client.get_goal_status_text())
        
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

        center = t.quaternion_from_euler(0,0,theta_user)
        self.br.sendTransform((x_goal,y_goal,0.0),
                        (center[0],center[1],center[2],center[3]),
                        rospy.Time.now(),
                        "behind_body",
                        "base_link")
        rospy.sleep(1)
        try:
            (behind_trans, behind_rot) = self._tf_Listener.lookupTransform("base_link","behind_body",rospy.Time(0))
            self.nav_move_base(behind_trans[0],behind_trans[1],0,behind_rot[0],behind_rot[1],behind_rot[2],behind_rot[3])

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            rospy.logwarn(f"{self.string_header}Transform not published yet :{e}")

if __name__ == "__main__":
    rospy.init_node("follow_user",log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(),ARIHeadFollower)