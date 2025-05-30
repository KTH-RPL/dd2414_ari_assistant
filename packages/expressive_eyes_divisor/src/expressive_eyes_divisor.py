#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CompressedImageSplitter:
    def __init__(self):
        rospy.init_node('expressive_eyes_divisor', anonymous=False)

        # Params
        self.input_topic = "/robot_face/image_raw/compressed"
        self.left_output_topic = "/robot_face/left_eye"
        self.right_output_topic = "/robot_face/right_eye"
        self.output_topic = "/robot_face"

        self.bridge = CvBridge()

        # Publishers
        self.out_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)

        # Subscriber
        rospy.Subscriber(self.input_topic, CompressedImage, self.callback)

        rospy.loginfo("Expressive Eyes Divisor Node Started")
        rospy.spin()

    def callback(self, msg):
        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            rospy.logwarn("Failed to decode image")
            return

        all_image = cv_image

        # Convert to ROS Image messages
        all_msg = self.bridge.cv2_to_imgmsg(all_image, encoding="bgr8")
        

        # Copy header info
        all_msg.header = msg.header

        # Publish
        self.out_pub.publish(all_msg)

if __name__ == '__main__':
    try:
        CompressedImageSplitter()
    except rospy.ROSInterruptException:
        pass