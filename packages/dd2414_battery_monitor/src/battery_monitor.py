#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mm11_msgs.msg import Power

class BatteryMonitor:
    def __init__(self):
        rospy.init_node("battery_monitor")
        
        self.warning_pub = rospy.Publisher("/battery_warning", String, queue_size=10)
        rospy.Subscriber("/power_status", Power, self.power_status_callback)
        
    def power_status_callback(self, msg):
        #rospy.loginfo(f"Battery level at: {msg.charge}")
        if msg.charge < 100 and not msg.is_connected:
            rospy.loginfo(f"Battery level at: {msg.charge}")
            self.warning_pub.publish("Battery is low!")
    
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    monitor = BatteryMonitor()
    monitor.run()