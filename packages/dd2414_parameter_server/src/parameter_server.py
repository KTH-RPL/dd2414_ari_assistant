#!/usr/bin/env python

import rospy
from dynamic_reconfigure.client import Client

def update_inflation_radius():
    rospy.init_node('inflation_radius_updater')
    header_string = "[PARAM. SERVER  ]:"

    rospy.loginfo(header_string + "Initialized")
    # Create clients for both global and local inflation layers
    global_client = Client("/move_base/global_costmap/inflation_layer", timeout=5)
    global_robot_radius = rospy.get_param("/move_base/global_costmap/robot_radius",0.5)
    local_client = Client("/move_base/local_costmap/inflation_layer", timeout=5)
    local_robot_radius = rospy.get_param("/move_base/local_costmap/robot_radius",0.5)

    # Set new values for the inflation radius
    new_radius = 0.7  # Example: update to 0.7 meters

    global_client.update_configuration({"inflation_radius": global_robot_radius})
    rospy.loginfo(header_string + "Updated global inflation radius to %.2f", global_robot_radius)

    local_client.update_configuration({"inflation_radius": local_robot_radius})
    rospy.loginfo(header_string + "Updated local inflation radius to %.2f", local_robot_radius)
    rospy.loginfo(header_string + "Done")


if __name__ == "__main__":
    update_inflation_radius()