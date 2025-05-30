#!/usr/bin/env python

import rospy
from dynamic_reconfigure.client import Client
from pal_zoi_detector.msg import CurrentZoI
from std_srvs.srv import Empty
class ParamServer:

    def __init__(self):
        rospy.init_node('inflation_radius_updater')
        self.curr_zoi = rospy.Subscriber("/current_zone_of_interest",CurrentZoI,self.cb_current_zoi)
        self.rate = rospy.Rate(1)
        self.header_string = "[PARAM. SERVER  ]:"

        self.global_client = Client("/move_base/global_costmap/inflation_layer", timeout=5)
        self.local_client = Client("/move_base/local_costmap/inflation_layer", timeout=5)
        self.local_rgbd = Client("/move_base/local_costmap/stvl_rgbd_layer", timeout=5)
        self.service_proxy = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
        self.current_status = False
        self.currently_at_door = False
        self.counter = 0

        
    def update_inflation_radius(self):

        rospy.loginfo(self.header_string + "Initialized")
        # Create clients for both global and local inflation layers
        self.global_robot_radius = rospy.get_param("/move_base/global_costmap/robot_radius",0.5)
        self.local_robot_radius = rospy.get_param("/move_base/local_costmap/robot_radius",0.5)

        # Set new values for the inflation radius
        new_radius = 0.7  # Example: update to 0.7 meters

        self.global_client.update_configuration({"inflation_radius": self.global_robot_radius})
        rospy.loginfo(self.header_string + "Updated global inflation radius to %.2f", self.global_robot_radius)

        self.local_client.update_configuration({"inflation_radius": self.local_robot_radius})
        rospy.loginfo(self.header_string + "Updated local inflation radius to %.2f", self.local_robot_radius)

        self.mark_threshold = 10
        self.local_rgbd.update_configuration({"mark_threshold":self.mark_threshold})
        rospy.loginfo(self.header_string + "Updated local rgbd mark threshold to %.2f", self.mark_threshold)
        rospy.loginfo(self.header_string + "Done")

    def dynamic_update(self):
        #if there is a change in status perform the param server update
        if self.current_status != self.currently_at_door:
            rospy.logdebug(f"{self.header_string}Updated Server: Currently at door {self.currently_at_door}")
            if self.currently_at_door:
                self.global_client.update_configuration({"inflation_radius": 0.1})
                self.local_client.update_configuration({"inflation_radius": 0.1})
                try:
                    response = self.service_proxy()  # Call with argument(s)
                    rospy.logdebug(f"{self.header_string}Service response:", response)
                except rospy.ServiceException as e:
                    rospy.logdebug(f"{self.header_string}Service call failed:", e)
            else:
                self.global_client.update_configuration({"inflation_radius": self.global_robot_radius})
                self.local_client.update_configuration({"inflation_radius": self.local_robot_radius})
            
            self.current_status = self.currently_at_door

        if self.counter == 5:
            try:
                response = self.service_proxy()  # Call with argument(s)
                rospy.logdebug(f"{self.header_string}Service response:", response)
            except rospy.ServiceException as e:
                rospy.logdebug(f"{self.header_string}Service call failed:", e)
            self.counter = 0
        else:
            self.counter = self.counter + 1
                


        self.rate.sleep()

    def cb_current_zoi(self,msg):
        for zoi in msg.zois:
            if "door_" in zoi:
                self.currently_at_door = True
                return
            
        self.currently_at_door = False

if __name__ == "__main__":
    node = ParamServer()
    node.update_inflation_radius()
    while not rospy.is_shutdown():
        node.dynamic_update()
