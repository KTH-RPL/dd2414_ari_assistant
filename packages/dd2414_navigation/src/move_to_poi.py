
#!/usr/bin/env python3

import rospy

import actionlib
from dd2414_status_update import StatusUpdate
from pal_composite_navigation_msgs.msg import GoToFloorPOIAction, GoToFloorPOIGoal
from visualization_msgs.msg import InteractiveMarkerInit

class MoveToPOI:
    def __init__(self):
        self.string_header = "[MOVE_TO_POI    ]:"
        rospy.loginfo(self.string_header + "Initializing")
        self.poi_dict = {}
        self._ac_navigation = actionlib.SimpleActionClient('/composite_navigation',GoToFloorPOIAction)
        self._sub_map_poi = rospy.Subscriber('/poi_marker_server/update_full',InteractiveMarkerInit, self.map_poi_conversion)
    
    def action(self,goal):

        if goal.goal in self.poi_dict:
            meta = GoToFloorPOIGoal()
            meta.poi = goal.goal #This is assigning the waypoint for the GoToFloorPOI GOal
            self._ac_navigation.send_goal(meta)
            timeout = rospy.Duration(20)
            wait = self._ac_navigation.wait_for_result(timeout)
            status = self._ac_navigation.get_state()
            result = self._ac_navigation.get_result()
            rospy.logdebug(self.string_header + str(wait))
            rospy.logdebug(self.string_header + str(status))
            rospy.logdebug(self.string_header + str(result))
            if status == 3:
                return "Success"
            elif status == 0:
                return "Working"
            else:
                return "Failure"
        else:
            rospy.loginfo(self.string_header + "Waypoint not found")
            return "Failure"
        
    def map_poi_conversion(self,data):
        marker_dict = {marker.name : marker for marker in data.markers}
        self.poi_dict = marker_dict
        rospy.logdebug(self.string_header + str(list(marker_dict.keys())))
        
if __name__ == '__main__':
    rospy.init_node('move_to_poi',log_level=rospy.INFO)
    server = StatusUpdate(rospy.get_name(),MoveToPOI)

    rospy.sleep(10)
    rospy.loginfo("READY")
    rospy.spin()