#!/usr/bin/env python3

import rospy
from std_msgs.msg import String  # Change to actual message type
from hri_msgs.msg import LiveSpeech
import actionlib
import dd2414_brain_v2.msg as brain
from actionlib_msgs.msg import GoalStatus

class SpeechListener:
    def __init__(self):
        rospy.init_node('speech_listener_node')

        # Action client for find_speaker
        self.client = actionlib.SimpleActionClient('/find_speaker', brain.BrainAction)
        rospy.loginfo("Waiting for /find_speaker action server...")
        self.client.wait_for_server()
        rospy.loginfo("/find_speaker action server available.")

        # Subscribe to speech topic
        self.speech_sub = rospy.Subscriber(
            '/humans/voices/anonymous_speaker/speech',
            LiveSpeech,
            self.speech_callback
        )

        self.ticker = rospy.Timer(rospy.Duration(5.0), self.tick_goal, oneshot=False)  # every 5 seconds
        self.goal_active = False
        self.goal_succeeded = False

    def speech_callback(self, msg):
        if msg.final.strip() and not self.goal_succeeded:
            rospy.loginfo(f"Detected final speech: {msg.final}")
            self.send_goal()

    def send_goal(self):
        if not self.goal_active:
            goal = brain.BrainGoal()  # Add fields if needed
            rospy.loginfo("Sending goal to /find_speaker...")
            self.client.send_goal(goal, done_cb=self.done_callback)
            self.goal_active = True

    def tick_goal(self, event):
        if self.goal_active and not self.goal_succeeded:
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("find_speaker action succeeded.")
                self.goal_succeeded = True
                self.goal_active = False
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]:
                rospy.logwarn(f"Goal failed with state {state}, retrying...")
                self.goal_active = False
            else:
                rospy.loginfo("Goal still in progress or pending, waiting...")

            # Resend if not succeeded
            if not self.goal_active and not self.goal_succeeded:
                self.send_goal()

    def done_callback(self, state, result):
        rospy.loginfo(f"Action completed with state: {state}")
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
            self.goal_succeeded = True
        else:
            rospy.logwarn("Goal did not succeed, will retry.")

        self.goal_active = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SpeechListener()
        node.run()
    except rospy.ROSInterruptException:
        pass
