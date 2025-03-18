#!/usr/bin/env python3

import rospy

from dd2414_status_update import StatusUpdate

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

        

class MoveBase:
    def __init__(self):
        self.move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.location_dict = {"desk" : (1.0     , 1.0),
                              "table" :(0.0    , 0.0)}
        
    def action(self,goal):
        position = self.location_dict[goal.goal]
        result = self.nav_move_base(position[0],position[1])
        return result

    def preempted(self):
        pass
    
#    def cb_done (self,status,result):
#        pass
#    def cb_feedback(self,feedback):
#        pass
#    def cb_active(self):
#        pass
    def nav_move_base(self,req_x, req_y):
        self.move_client.wait_for_server()
        rospy.loginfo("Move base client ready")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = req_x
        goal.target_pose.pose.position.y = req_y
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal")
        self.move_client.send_goal(goal)
        timeout = rospy.Duration(20)
        wait = self.move_client.wait_for_result(timeout)
        status = self.move_client.get_state()
        #result = self.move_client.get_result()
        #rospy.loginfo(wait)
        #rospy.loginfo(status)
        #rospy.loginfo(result)
        if status == 3:
            return "Success"
        elif status == 0:
            return "Working"
        else:
            return "Failure"

if __name__ == '__main__':
    rospy.init_node('nav_move_base_server')
    server = StatusUpdate(rospy.get_name(),MoveBase)
    rospy.spin()