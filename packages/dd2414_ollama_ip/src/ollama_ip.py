#!/usr/bin/env python
import rospy
import socket
from std_msgs.msg import String

class OllamaIP:
    def __init__(self):
        rospy.init_node("ollama_ip")
        self.llm_ip = ""
        self.ollama_ip_pub = rospy.Publisher("/llm_ip",String,queue_size=15)
        rospy.sleep(10)
        self.run()

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))  # No se envían datos realmente
        self.llm_ip = str(s.getsockname()[0])
        msg = String()
        msg.data = self.llm_ip
        self.ollama_ip_pub.publish(msg)
        rospy.loginfo(msg)
        s.close()

if __name__ == '__main__':
    try:
        OllamaIP()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("OllamaIP: ROS Node interrupted.")
