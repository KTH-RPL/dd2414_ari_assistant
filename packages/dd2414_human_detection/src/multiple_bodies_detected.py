#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from hri_msgs.msg import IdsList, NormalizedRegionOfInterest2D
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import torch  # Import torch for YOLOv5

class HumanDetectionNode:
    def __init__(self):
        rospy.init_node('human_detection_node', anonymous=True)

        self.bridge = CvBridge()

        # Subscribe to camera image
        self.image_sub = rospy.Subscriber('/image', Image, self.image_callback)

        # Publishers
        self.body_ids_pub = rospy.Publisher('/humans/bodies/tracked', IdsList, queue_size=10)

        # Load YOLOv5 model
        YOLO_PATH = '/home/laura/catkin_ws/src/dd2414/dd2414_human_detection/src/yolov5/yolov5s.pt'

        
        if not os.path.isfile(YOLO_PATH):
            rospy.logerr(f"YOLO file not found at: {YOLO_PATH}")
        else:
            rospy.loginfo(f"YOLO file found at: {YOLO_PATH}")

        # Load the YOLOv5 model using PyTorch
        self.net = torch.hub.load('ultralytics/yolov5', 'custom', path=YOLO_PATH, force_reload=True)
        self.conf_threshold = 0.5  # Confidence threshold
        self.nms_threshold = 0.4     # Non-maximum suppression threshold

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Detect humans using YOLOv5
        detected_bodies = self.detect_bodies(cv_image)

        # Publish detected body IDs
        body_ids_msg = IdsList()
        for i, (x_min, y_min, x_max, y_max) in enumerate(detected_bodies):
            body_id = f"body_{i}"
            body_ids_msg.ids.append(body_id)

            # Create ROI message
            roi_msg = NormalizedRegionOfInterest2D()
            roi_msg.x_min = x_min / cv_image.shape[1]
            roi_msg.y_min = y_min / cv_image.shape[0]
            roi_msg.x_max = x_max / cv_image.shape[1]
            roi_msg.y_max = y_max / cv_image.shape[0]

            # Crop detected body
            cropped_image = cv_image[y_min:y_max, x_min:x_max]
            cropped_msg = self.bridge.cv2_to_imgmsg(cropped_image, "bgr8")

            # Create dynamic topic names
            roi_topic = f'/humans/bodies/{body_id}/roi'
            cropped_topic = f'/humans/bodies/{body_id}/cropped'

            # Dynamically create publishers
            roi_pub = rospy.Publisher(roi_topic, NormalizedRegionOfInterest2D, queue_size=10)
            cropped_img_pub = rospy.Publisher(cropped_topic, Image, queue_size=10)

            # Publish messages
            roi_pub.publish(roi_msg)
            cropped_img_pub.publish(cropped_msg)

        self.body_ids_pub.publish(body_ids_msg)

    def detect_bodies(self, image):
        results = self.net(image)
        detections = results.xyxy[0]  # Get detections in (x1, y1, x2, y2, confidence, class) format

        boxes = []
        for *box, conf, cls in detections:
            if conf > self.conf_threshold and int(cls) == 0:  # Class ID 0 is 'person'
                x_min, y_min, x_max, y_max = map(int, box)
                boxes.append([x_min, y_min, x_max, y_max])

        return boxes

def main():
    node = HumanDetectionNode()
    rospy.spin()

if __name__ == '__main__':
    main()
