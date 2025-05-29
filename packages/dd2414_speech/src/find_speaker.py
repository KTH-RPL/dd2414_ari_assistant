#!/usr/bin/env python3

import rospy
import actionlib
import tf2_ros as tf
import numpy as np
import pyhri
from std_msgs.msg import Int32, Bool, String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from dd2414_status_update import StatusUpdate
import dd2414_brain_v2.msg as brain

class FindSpeakerActionServer:
    
    def __init__(self):

        self.result = brain.BrainResult()
        self.result.result = ""

        self.directions = []
        self.buffer_duration = rospy.Duration(5.0) # Keep last 5 seconds of data
        self.speaking_in_progress = False
        self.turning_to_speech = False
        self.finding_speaker_active = False
        self.turning_goal = MoveBaseGoal()

        self.hri_listener = pyhri.HRIListener()

        self.speech_sub = rospy.Subscriber('/humans/voices/anonymous_speaker/is_speaking', Bool, self.speech_cb, queue_size=10)
        rospy.logdebug(f"[FIND_SPEAKER   ]:Subscribed to {self.speech_sub.resolved_name}")

        self.person_found_sub = rospy.Subscriber('person_looking_at_robot', String, self.person_found_cb, queue_size=10)
        rospy.logdebug(f"[FIND_SPEAKER   ]:Subscribed to {self.person_found_sub.resolved_name}")

        self.direction_sub = rospy.Subscriber('/audio/sound_direction', Int32, self.direction_cb)
        rospy.logdebug(f"[FIND_SPEAKER   ]:Subscribed to {self.direction_sub.resolved_name}")

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("[FIND_SPEAKER   ]:Initialized")
        self.string_header = "[FIND_SPEAKER   ]:"
        
    def action(self,goal):
        rospy.loginfo("[FIND_SPEAKER   ]:Request to look at speaker received") #CHANGE
        self.person_found = False
        
        
        if(not self.finding_speaker_active):
            if(len(self.directions) > 0):
                self.finding_speaker_active = True
                median_direction = np.median([d for (_, d) in self.directions])
                self.result.result = "Running"
                self.rotate_to(median_direction)
                self.directions = []
                
            else:
                rospy.loginfo("[FIND_SPEAKER   ]:Previous speech could not be localized")
                self.result.result = "Success"
        
        return self.result
    
    def preempted(self):
        rospy.logdebug("[FIND_SPEAKER   ]:Goal preempted") 
        self.move_base_client.cancel_goal(self.turning_goal)
        self.finding_speaker_active = False
        self.result.result = "Failure" # Mark the goal as preempted
        return self.result        
    
    def rotate_to(self, direction):

        rospy.loginfo(f"[FIND_SPEAKER   ]:Sending request to rotate in direction {direction}")
        
        self.turning_to_speech = True
        if direction >= 0 :
            rotation_rad = np.deg2rad(180+10)
        else:
            rotation_rad = np.deg2rad(180-10)
        #rotation_rad = np.deg2rad(-2*direction)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()

        quat = quaternion_from_euler(0, 0, rotation_rad)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        self.turning_goal = goal

        self.move_base_client.send_goal(goal, done_cb=self.rotating_done_cb,)

    def rotating_done_cb(self, state, result):
        self.turning_to_speech = False

        # Wait to see if body is found during or after rotation
        rospy.sleep(2)
        if(self.finding_speaker_active):
            rospy.loginfo("[FIND_SPEAKER   ]:Finished turning towards speech, could not find body") #CHANGE
            self.result.result = "Failure"
            self.finding_speaker_active = False

        rospy.loginfo("[FIND_SPEAKER   ]:Finished turning towards speech")

    # When body looking at ari is detected, turn to body
    def person_found_cb(self, data):
        if(data.data != "" and self.finding_speaker_active):
            rospy.loginfo("[FIND_SPEAKER   ]:Found body looking at ari")
            self.finding_speaker_active = False
            self.result.result = "Success"
            
            if(self.turning_to_speech):
                self.move_base_client.cancel_goal()
                self.turning_to_speech = False

            goal = MoveBaseGoal()
            body = self.hri_listener.bodies.get(data.data)

            try:
                transform = body.transform("base_link").transform

                target_x = transform.translation.x
                target_y = transform.translation.y

                yaw = np.arctan2(target_y, target_x)
                quat = quaternion_from_euler(0, 0, yaw)

                goal = MoveBaseGoal()

                goal.target_pose.header.frame_id = 'base_link'
                
                goal.target_pose.pose.orientation.x = quat[0]
                goal.target_pose.pose.orientation.y = quat[1]
                goal.target_pose.pose.orientation.z = quat[2]
                goal.target_pose.pose.orientation.w = quat[3]

                goal.target_pose.header.stamp = rospy.Time.now()

                self.turning_goal = goal
                self.move_base_client.send_goal(goal)
                #wait = self.move_base_client.wait_for_result()
                rospy.logdebug("[FIND_SPEAKER   ]:Turning to found body")          
            except:
                rospy.loginfo("[FIND_SPEAKER   ]:Could not transform body frame") #CHANGE

            return self.result

    # Check for speech, always active
    def speech_cb(self, data):
        if(data.data):
            rospy.logdebug("[FIND_SPEAKER   ]:Speech started, recording direction")
            if(not self.speaking_in_progress):
                self.directions = []
            self.speaking_in_progress = True
            return
        else:
            rospy.logdebug("[FIND_SPEAKER   ]:Speech stopped, stopped recording direction")
            self.speaking_in_progress = False
    
    # Save directions, active during speech
    def direction_cb(self, data):
        now = rospy.Time.now()

        if(self.speaking_in_progress):
            direction = data.data
            if(direction == 180):
                return
            elif(direction < -90):
                direction = (-direction) - 180
            elif(direction > 90):
                direction = (-direction) + 180

            self.directions.append((now, direction))

        # Remove old entries outside buffer window
        self.directions = [
            (t, d) for (t, d) in self.directions
            if now - t <= self.buffer_duration
        ]

if __name__ == '__main__':

    try:
        # Initialize the node
        rospy.init_node('find_speaker', anonymous=False,log_level=rospy.INFO)
        server = StatusUpdate(rospy.get_name(),FindSpeakerActionServer)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

