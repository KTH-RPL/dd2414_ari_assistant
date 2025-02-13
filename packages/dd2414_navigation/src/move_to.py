#!/usr/bin/env python3

import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def nav_move_base(req_x, req_y):
    move_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    move_client.wait_for_server()
    rospy.loginfo("Move base client ready")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = req_x
    goal.target_pose.pose.position.y = req_y
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal")
    move_client.send_goal(goal)
    wait = move_client.wait_for_result()
    return move_client.get_result()

if __name__ == '__main__':
    req_position_x = float(rospy.get_param('pos_x'))
    req_position_y = float(rospy.get_param('pos_y'))
    try:
        rospy.init_node('nav_move_base_client')
        result = nav_move_base(req_position_x, req_position_y)
        if result:
            rospy.loginfo("Arrived at target!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Finished.")