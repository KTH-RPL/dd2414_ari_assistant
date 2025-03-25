#!/usr/bin/env python3


import rospy
import math
import actionlib
import numpy as np
import dd2414_brain_v2.msg as brain

from tf import TransformListener
from tf import transformations as t
from tf import TransformBroadcaster as tf_B
from tf import LookupException,ExtrapolationException,ConnectivityException
from pyhri import HRIListener
from tf2_ros import Buffer
from actionlib import SimpleActionClient
from hri_msgs.msg import LiveSpeech
from control_msgs.msg import FollowJointTrajectoryActionGoal, PointHeadActionGoal
from geometry_msgs.msg import PointStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dd2414_status_update import StatusUpdate
from pal_interaction_msgs.msg import TtsAction, TtsGoal


class ARIHeadFollower:
    def __init__(self):
        self.hri_listener = HRIListener()
        self.rate  = rospy.Rate(1)
        self.stop  = False
        self.br    = tf_B()
        self._tf_buffer = Buffer()
        self._tf_Listener = TransformListener()

        # Publisher for head movement
        self.look_at_pub  = rospy.Publisher("/look_at",PointStamped,queue_size=1)
        self.look_at_goal = rospy.Publisher("/head_controller/point_head_action/goal",PointHeadActionGoal,queue_size=1)
        self.head_pub = rospy.Publisher('/head_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)

        self.asr_sub      = rospy.Subscriber('/humans/voices/anonymous_speaker/speech',LiveSpeech, self.asr_result)
        self.tts_client   = SimpleActionClient("/tts", TtsAction)
        self.tts_client.wait_for_server()
        self.language = "en_US"

        # Joint limits
        self.head_1_min, self.head_1_max = -1.2, 1.2
        self.head_2_min, self.head_2_max = -0.2, 0.4

        self.track_user

    def asr_result(self,msg):
        sentence = msg.final
        rospy.loginfo(f"Understood sentence: {sentence}")
        
        if sentence.lower() == "stop":
            self.stop = True
            rospy.loginfo("Stoping Following behavior")
            self.tts_output("Stopping Follow User behavior")
        
        #if sentence.lower() == "follow":
        #    rospy.loginfo("Starting Following behavior")
        #    self.tts_output("Starting Follow User behavior")
        #    self.stop = False

    def tts_output(self,answer):
        self.tts_client.cancel_goal()
        goal = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text = str(answer)
        self.tts_client.send_goal_and_wait(goal)

    def map_range(self, value, in_min, in_max, out_min, out_max):
        """ Maps a value from one range to another """
        return out_min + (float(value - in_min) / float(in_max - in_min) * (out_max - out_min))
    
    def nav_move_base(self,x,y,z,rx,ry,rz,rw):
        move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        move_client.wait_for_server()
        rospy.loginfo("Move base client ready")

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

        rospy.loginfo("Sending goal")
        move_client.send_goal(goal)
        move_client.wait_for_result()

        rospy.loginfo(f"Goal state:{move_client.get_state()}")
        rospy.loginfo(move_client.get_goal_status_text())
        
        return move_client.get_state()
    
    def round_number(self,x):
        return 1.5 if x >= 0 else -1.5
    
    def approach(self,body_frame):
        #x = self.round_number(x)
        
        (t_bodies,r_bodies) = self._tf_Listener.lookupTransform("base_link",body_frame,rospy.Time(0))
        x_user = t_bodies[0]
        y_user = t_bodies[1]

        radius = math.sqrt(x_user**2 + y_user**2) - 1.0
        theta_user = math.atan2(y_user,x_user)

        x_goal = radius * math.cos(theta_user)
        y_goal = radius * math.sin(theta_user)

        #(t_bodies,r_bodies) = self._tf_Listener.lookupTransform(body_frame,"base_link",rospy.Time(0))

        #x_ari = t_bodies[0]
        #y_ari = t_bodies[1]
        #theta = math.atan2(y_ari,x_ari)
        #x_goal = 1.0 * np.cos(theta)
        #y_goal = 1.0 * np.sin(theta)
        #theta_goal = theta + np.pi

        #print("X: "+ str(x) + " Y: " + str(y) + " Rot: " + str(z))
        center = t.quaternion_from_euler(0,0,theta_user)
        #print(theta_user/np.pi*180)
        #print(theta_goal/np.pi*180)
        #print(center)
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
            print(e)
            print("Transform not published yet")

    def action (self,goal):
        return self.track_user()

    def preempted(self):
        #Cancel CURRENT GOAL
        pass

    def track_user(self):

        #while not rospy.is_shutdown(): #and not self.stop:            
            try:
                if len(self.hri_listener.bodies) > 0 and (not self.stop): # are bodies detected?
                    # Get transform from head_camera_link to detected body
                    #transform = self.tf_buffer.lookup_transform('base_link', 'head_camera_link', rospy.Time(0), rospy.Duration(1.0))
                    #trans = transform.transform.translation

                    bodies = list(self.hri_listener.bodies.values())[0]
                    
                    transform_bodies = bodies.transform()

                    (t_bodies,r_bodies) = self._tf_Listener.lookupTransform("base_link",bodies.frame,rospy.Time(0))

                    # Convert position to joint angles
                    if t_bodies[0] <= 1.5:

                        #target = PointStamped(point=Point(x=t_bodies[0], y=t_bodies[1], z=(t_bodies[2])), header=t_bodies.header)
                        #self.look_at_pub.publish(target)

                        head_goal = PointHeadActionGoal()
                        head_goal.goal.target.header.frame_id = "base_link"
                        point = Point(x=t_bodies[0], y=t_bodies[1], z=t_bodies[2])
                        head_goal.goal.pointing_axis.x = 0.0
                        #head_goal.goal.pointing_axis.y = 0.0
                        head_goal.goal.pointing_frame = "sellion_link"

                        angle = math.atan2(point.y,point.x)
                        if angle > 1.4:
                            facing = t.quaternion_from_euler(0,0,np.pi/2)
                            result = self.nav_move_base(0,0,0,facing[0],facing[1],facing[2],facing[3])
                            point = Point(x=1.0, y=0.0, z=1.5)
                        elif angle < -1.4:
                            facing = t.quaternion_from_euler(0,0,-np.pi/2)
                            result = self.nav_move_base(0,0,0,facing[0],facing[1],facing[2],facing[3])
                            point = Point(x=1.0, y=-0.0, z=1.5)

                        head_goal.goal.target.point = point
                        self.look_at_goal.publish(head_goal)

                        #head_1_pos = self.map_range(t_bodies[1], -np.pi/2, np.pi/2, self.head_1_min, self.head_1_max)
                        #head_2_pos = 0 #self.map_range(t_bodies[2], 1.4, 1.8, self.head_2_min, self.head_2_max)

                        # Publish movement command    
                        #goal_msg = FollowJointTrajectoryActionGoal()
                        #goal_msg.goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
                        #point = JointTrajectoryPoint()
                        #point.positions = [head_1_pos, head_2_pos]
                        #point.time_from_start = rospy.Duration(1.0)

                        #if head_1_pos >=0.8 and head_1_pos < 2:
                        #    facing = t.quaternion_from_euler(0,0,np.pi/2)
                        #    point.positions = [0, 0]
                        #    goal_msg.goal.trajectory.points.append(point)
                        #    result = self.nav_move_base(0,0,0,facing[0],facing[1],facing[2],facing[3])

                        #elif head_1_pos <=-0.8 and head_1_pos > -2:
                        #    facing = t.quaternion_from_euler(0,0,-np.pi/2)
                        #    point.positions = [0, 0]
                        #    goal_msg.goal.trajectory.points.append(point)
                        #    result = self.nav_move_base(0,0,0,facing[0],facing[1],facing[2],facing[3])

                        #else:
                        #    goal_msg.goal.trajectory.points.append(point)

                        #self.head_pub.publish(goal_msg)

                    else:
                        self.approach(bodies.frame)
                result = brain.BrainResult()
                
                if self.stop:
                    self.stop = False
                    result.result = "Success"
                    return result

                result.result = "Working"        
                return result
            
            except Exception as e:
                rospy.logwarn(f"Could not transform: {e}")
                result = brain.BrainResult()
                result.result = "Working"
                return result


if __name__ == "__main__":
    rospy.init_node("follow_user")
    server = StatusUpdate(rospy.get_name(),ARIHeadFollower)














"""
import rospy
from tf2_ros import Buffer
from tf import TransformListener

from pyhri import HRIListener
from std_msgs.msg import String
from actionlib import SimpleActionClient

from geometry_msgs.msg import Point, PointStamped
from pal_interaction_msgs.msg import TtsAction, TtsGoal


class FollowUser:
    def __init__(self):
        self.hri_listener = HRIListener()
        self.look_at_pub  = rospy.Publisher("/look_at",PointStamped,queue_size=1)
        #self.asr_sub      = rospy.Subscriber('/humans/voices/anonymous_speaker/speech',LiveSpeech, self.asr_result)
        #self.tts_client   = SimpleActionClient("/tts", TtsAction)
        #self.tts_client.wait_for_server()
        #self.language = "en_US"
        self.rate     = rospy.Rate(1)
        self.stop     = True
        
        self._tf_buffer = Buffer()
        self._tf_Listener = TransformListener()

    def asr_result(self,msg):
        sentence = msg.final
        rospy.loginfo(f"Understood sentence: {sentence}")
        
        if sentence.lower() == "stop":
            self.stop = True
            rospy.loginfo("Stoping Following behavior")
            self.tts_output("Stopping Follow User behavior")
        
        if sentence.lower() == "follow user":
            rospy.loginfo("Starting Following behavior")
            self.tts_output("Starting Follow User behavior")
            self.stop = False

    def tts_output(self,answer):
        self.tts_client.cancel_goal()
        goal = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text = str(answer)
        self.tts_client.send_goal_and_wait(goal)
    
    def follow_user(self):

        pass

    def run(self):
        while not rospy.is_shutdown():
            #if not self.stop:
                if len(self.hri_listener.bodies) > 0: # are there faces detected?
                    
                    bodies = list(self.hri_listener.bodies.values())[0]
                    #person = self.hri_listener.tracked_persons.get(id)

                    #face = list(self.hri_listener.faces.values())[0] # look at first face in the list
                    #print(f"looking at {face.id}")

                    try:
                        transform_bodies = bodies.transform()
                        trans_bodies = transform_bodies.transform.translation
                        #print("Coordinates:")
                        #print(trans_bodies)
                        #print("")

                        #T = face.transform()
                        #trans_face = T.transform.translation

                        (t_bodies,r_bodies) = self._tf_Listener.lookupTransform("base_link",bodies.frame,rospy.Time(0))
                        print("Translation:")
                        print(t_bodies)
                        print("Rotation:")
                        print(r_bodies)
                        print("")
                        t_b = t_bodies
                        #print("Bodies:")
                        #print(trans_bodies)
                        #print("Bodies 2:")
                        #print(t_bodies)
                        #print("")
                        target = PointStamped(point=Point(x=t_bodies[0], y=t_bodies[1], z=(t_bodies[2]-0.6)), header=transform_bodies.header)
                        #self.look_at_pub.publish(target)
                    
                    except Exception as e:
                        rospy.logwarn(f"Could not transform face position: {e}")
                

if __name__=="__main__":
    rospy.init_node("follow_user")
    follow = FollowUser()
    follow.run()

"""