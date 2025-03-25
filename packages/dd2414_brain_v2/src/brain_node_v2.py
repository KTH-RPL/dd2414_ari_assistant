
import rospy
from std_msgs.msg import String
import json

import actionlib
import dd2414_brain_v2.msg as brain

class Brain:
    def __init__(self):
        rospy.loginfo("Brain Start Initializing")

        self.intent_sub=rospy.Subscriber(rospy.get_param("intent_topic","~intent"),String,self.intent_cb)


        rospy.loginfo("Brain Finished Initializing")

    def intent_cb(self,intent):
        

if __name__ == '__main__':
    rospy.init_node('brain',anonymous=False)
    node = Brain()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        node.run()
        rate.sleep()

    node.shutdown()