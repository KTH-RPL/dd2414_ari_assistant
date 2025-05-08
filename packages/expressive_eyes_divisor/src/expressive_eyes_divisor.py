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
#        self.left_pub = rospy.Publisher(self.left_output_topic, Image, queue_size=1)
#        self.right_pub = rospy.Publisher(self.right_output_topic, Image, queue_size=1)
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

#        height, width, _ = cv_image.shape
#        mid = width // 2

        # Split image vertically
#        left_image = cv_image[:, :mid]
#        right_image = cv_image[:, mid:]
        all_image = cv_image

        # Convert to ROS Image messages
#        left_msg = self.bridge.cv2_to_imgmsg(left_image, encoding="bgr8")
#        right_msg = self.bridge.cv2_to_imgmsg(right_image, encoding="bgr8")
        all_msg = self.bridge.cv2_to_imgmsg(all_image, encoding="bgr8")
        

        # Copy header info
#        left_msg.header = msg.header
#        right_msg.header = msg.header
        all_msg.header = msg.header

        # Publish
#        self.left_pub.publish(left_msg)
#        self.right_pub.publish(right_msg)
        self.out_pub.publish(all_msg)

if __name__ == '__main__':
    try:
        CompressedImageSplitter()
    except rospy.ROSInterruptException:
        pass