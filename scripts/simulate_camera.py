#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class SimulateCamera(object):

    def __init__(self):

        rospy.init_node("simulate_camera_node")
        self.publisher = rospy.Publisher(
            "camera/rgb/image_raw", Image, queue_size=10)
        self.rate = rospy.Rate(29)
        self.bridge = CvBridge()
        self.spin()

    def spin(self):
        rospy.loginfo("Starting camera simulation.")
        cv2.namedWindow("Camera Simulation", cv2.WINDOW_NORMAL)
        capture = cv2.VideoCapture(0)
        while not rospy.is_shutdown():
            if capture.isOpened():
                _, image = capture.read()
            else:
                image = np.random.randint(0, 255,
                    (360, 640, 3), dtype=np.uint8)
            image_message = self.bridge.cv2_to_imgmsg(
                image, "bgr8")
            cv2.imshow("Camera Simulation", image)
            cv2.waitKey(5)
            self.publisher.publish(image_message)
            self.rate.sleep()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    sc = SimulateCamera()

