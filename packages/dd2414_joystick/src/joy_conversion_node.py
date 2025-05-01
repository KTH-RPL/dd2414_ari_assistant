#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyConversion:
    def __init__(self):
        rospy.loginfo("[JOYSTICK       ]:Joy Conversion Initializing")
        self.joy_sub = rospy.Subscriber(rospy.get_param("~joy_out","/joy2"), Joy ,self.joy_cb)
        self.cmd_vel_pub = rospy.Publisher(rospy.get_param("~conversion_out","/cmd_vel2"),Twist,queue_size=10)
        
        self.last_time = rospy.Time.now().to_sec()

        self.timeout = rospy.get_param("timeout",5)

        self.last_msg = Twist()

        rospy.loginfo("[JOYSTICK       ]:Initialized")
        self.string_header = "[JOYSTICK       ]:"

    
    def joy_cb(self,Joy_data):

        cmd_vel = Twist()

        if abs(Joy_data.axes[0]) > 0.1 :
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = max(min(Joy_data.axes[0],0.5),-0.5)
        else:
            cmd_vel.linear.x =  max(min(Joy_data.axes[3],1.0),-1.0)
            cmd_vel.angular.z =  max(min(Joy_data.axes[2],1.0),-1.0)

        self.last_msg = cmd_vel
        self.last_time= rospy.Time.now().to_sec()
        #rospy.loginfo(cmd_vel)
        self.cmd_vel_pub.publish(cmd_vel)
    
    def checJoystickTimeout(self):
        now = rospy.Time.now().to_sec()

        if now - self.last_time > self.timeout and self.timeout != 0.0:
            self.cmd_vel_pub.publish(Twist())
            rospy.logwarn("Lost connection to the 'joy' topic. Stopping Robot")
        else:
            self.cmd_vel_pub.publish(self.last_msg)
    
    def shutdown(self):
        rospy.loginfo("Shutting Down node")
        self.cmd_vel_pub.publish(Twist)
    


if __name__ == '__main__':
    try:
        rospy.init_node("joy_convserion_node",log_level=rospy.INFO)
        node = JoyConversion()
        rate = rospy.Rate(10) #10Hz
        while not rospy.is_shutdown():
            node.checJoystickTimeout()
            rate.sleep()

        node.shutdown()
    except rospy.ROSInterruptException:
        rospy.loginfo("Joy Conversion Ended")