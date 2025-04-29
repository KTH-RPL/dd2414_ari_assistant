#!/usr/bin/env python3

import rospy

from dd2414_status_update import StatusUpdate

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations as t
import math
import numpy as np
import dd2414_brain_v2.msg as brain
        

class MoveBase:
    def __init__(self):
        self.move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.location_dict = {"desk" : (-0.85     , -0.65),
                              "table" :(-1.0    , -1.0),
                              "test" : (1.4029536753809344    , -0.5402147151304166),
                              "desk" : (-0.21023699455605094   , 0.08828129621303571)}
        self.rate = rospy.Rate(10)
        self.move_base_status = 3
        self.current_goal = None
        
    def action(self,goal):
        result = brain.BrainResult()

        if goal.goal and goal.goal in self.location_dict:

            if(self.current_goal != goal.goal and self.current_goal is not None):
                self.result = "Failure"
                self.preempted()
                
            else:
                position = self.location_dict[goal.goal]
                result.result = self.nav_move_base(position[0],position[1])

                if(self.current_goal != goal.goal and self.current_goal is not None):
                    self.result = "Failure"

            self.current_goal = goal.goal
        else:
            result.result = "Failure"

        self.rate.sleep()
        return result

    def preempted(self):
        self.move_client.cancel_all_goals()
        pass
    
#    def cb_done (self,status,result):
#        pass
#    def cb_feedback(self,feedback):
#        pass
#    def cb_active(self):
#        pass
    def nav_move_base(self,req_x, req_y):
        
        if(self.move_base_status != 0 and self.move_base_status != 1):
            self.move_client.wait_for_server()
            rospy.loginfo("Move base client ready")

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = req_x
            goal.target_pose.pose.position.y = req_y
            theta = math.atan2(req_y,req_x) + np.pi
            quat = t.quaternion_from_euler(0,0,theta)
            goal.target_pose.pose.orientation.x = quat[0]
            goal.target_pose.pose.orientation.y = quat[1]
            goal.target_pose.pose.orientation.z = quat[2]
            goal.target_pose.pose.orientation.w = quat[3]

            rospy.loginfo("Sending goal")
            self.move_client.send_goal(goal)
            timeout = rospy.Duration(20)
            #wait = self.move_client.wait_for_result(timeout)


        status = self.move_client.get_state()
        self.move_base_status = status
        result = self.move_client.get_result()
        #rospy.loginfo(wait)
        rospy.loginfo(f"Status: {status}")
        rospy.loginfo(f"Result: {result}")
        if status == 3:
            self.current_goal = None
            return "Success"
        elif status == 0 or status == 1:
            return "Working"
        else:
            self.current_goal = None
            return "Failure"

if __name__ == '__main__':
    rospy.init_node('nav_move_base_server')
    server = StatusUpdate(rospy.get_name(),MoveBase)
    rospy.spin()