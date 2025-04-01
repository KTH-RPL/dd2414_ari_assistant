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
        if msg.charge < 20 and not msg.is_connected:
            self.warning_pub.publish("Battery is low!")
    
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    monitor = BatteryMonitor()
    monitor.run()