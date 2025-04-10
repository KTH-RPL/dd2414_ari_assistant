#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from hri_msgs.msg import IdsList, NormalizedRegionOfInterest2D
from cv_bridge import CvBridge
import random
import string
import cv2
from ultralytics import YOLO

class HumanDetectionNode:
    def __init__(self):
        rospy.init_node('human_detection_node', anonymous=True,log_level=rospy.INFO)

        self.bridge = CvBridge()
        self.frame_counter = 0  # Add a frame counter

        # ROS Parameters
        self.conf_threshold = rospy.get_param("~conf_threshold", 0.25)

        # Subscribe to camera image
        self.image_sub = rospy.Subscriber('/head_front_camera/color/image_raw', Image, self.image_callback, queue_size=1)

        # Publishers
        self.body_ids_pub = rospy.Publisher("/humans/bodies/tracked", IdsList, queue_size=1)

        # Load YOLOv8 model
        self.model = YOLO("yolov8n.pt")

        # Dictionary to store track ID mappings
        self.track_id_mapping = {}

    def generate_random_id(self, length=5):
        """ Generate a random 5-letter ID. """
        return ''.join(random.choices(string.ascii_lowercase, k=length))

    def image_callback(self, msg):
        #rospy.loginfo("Image arrives at time stamp: " + rospy.Time.now())
        self.frame_counter += 1

        # Skip every 2nd frame (frame skipping)
        #if self.frame_counter % 10 != 0:
         #   return  # Skip this frame

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Resize image for faster processing (e.g., 640x480)
        cv_image_resized = cv2.resize(cv_image, (640, 480))

        # Detect bodies in the resized image
        detected_bodies = self.detect_bodies(cv_image_resized)
        rospy.loginfo(f"Raw detections: {detected_bodies}")

        # Prepare messages
        body_ids_msg = IdsList()
        roi_msgs = []
        cropped_msgs = []

        # Publish detected bodies
        for (x_min, y_min, x_max, y_max, track_id) in detected_bodies:
            # Generate a unique ID for each track ID if not already mapped
            if track_id not in self.track_id_mapping:
                self.track_id_mapping[track_id] = self.generate_random_id()  # Generate ID if new
            body_id = self.track_id_mapping[track_id]

            if body_id not in body_ids_msg.ids:
                body_ids_msg.ids.append(body_id)

            # Create ROI message
            roi_msg = NormalizedRegionOfInterest2D()
            roi_msg.xmin = x_min / cv_image.shape[1]
            roi_msg.ymin = y_min / cv_image.shape[0]
            roi_msg.xmax = x_max / cv_image.shape[1]
            roi_msg.ymax = y_max / cv_image.shape[0]
            roi_msgs.append((body_id, roi_msg))

            # Crop detected body
            cropped_image = cv_image[y_min:y_max, x_min:x_max]
            cropped_msg = self.bridge.cv2_to_imgmsg(cropped_image, "bgr8")
            cropped_msgs.append((body_id, cropped_msg))

        # Publish the list of detected body IDs
        rospy.loginfo(f"Detected body IDs: {body_ids_msg.ids}")
        self.body_ids_pub.publish(body_ids_msg)

        # Publish ROIs and cropped images
        #rospy.loginfo("Image arrives at time stamp: " + rospy.Time.now())
        self.publish_detections(roi_msgs, cropped_msgs)

    def publish_detections(self, roi_msgs, cropped_msgs):
        """ Publish detected ROIs and cropped images. """
        for body_id, roi_msg in roi_msgs:
            roi_msg.header.stamp = rospy.Time.now()
            topic = f"/humans/bodies/{body_id}/roi"
            roi_pub = rospy.Publisher(topic, NormalizedRegionOfInterest2D, queue_size=10)
            roi_pub.publish(roi_msg)

        for body_id, cropped_msg in cropped_msgs:
            cropped_msg.header.stamp = rospy.Time.now()
            topic = f"/humans/bodies/{body_id}/cropped"
            cropped_pub = rospy.Publisher(topic, Image, queue_size=10)
            cropped_pub.publish(cropped_msg)

        rospy.loginfo("Published ROI and cropped images for detected bodies.")

    def detect_bodies(self, image):
        """ Run YOLOv8 tracking and return detected bounding boxes with track IDs. """
        results = self.model.track(image, persist=True, conf=0.25, iou=0.2, classes=[0], show=False)
        detected_bodies = []
        if results[0].boxes is not None:
            for box in results[0].boxes.data.tolist():
                if len(box) >= 7:  # Ensure we have a track ID
                    print(box)
                    x_min, y_min, x_max, y_max, track_id, conf, cls = box[:7]
                    x_min, y_min, x_max, y_max = map(int, [x_min, y_min, x_max, y_max])

                    print(track_id)
                    print(conf)
                    if conf > self.conf_threshold and int(cls) == 0:  # Class ID 0 is 'person'
                        detected_bodies.append((x_min, y_min, x_max, y_max, track_id))
        
        return detected_bodies


def main():
    node = HumanDetectionNode()
    rospy.spin()

if __name__ == '__main__':
    main()
