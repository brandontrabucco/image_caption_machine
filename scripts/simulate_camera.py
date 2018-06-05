#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class SimulateCamera(object):

    def __init__(self, h=299, w=299, c=3):
        rospy.init_node("simulate_camera_node")
        self.publisher = rospy.Publisher(
            "camera/rgb/image_raw", Image, queue_size=10)
        self.rate = rospy.Rate(29)
        self.bridge = CvBridge()
        self.spin(h, w, c)

    def spin(self, height, width, channels):
        rospy.loginfo("Starting camera simulation.")
        while not rospy.is_shutdown():
            image = self.bridge.cv2_to_imgmsg(
                np.random.randint(0, 255, 
                    (height, width, channels), dtype=np.uint8),
                "bgr8")
            self.publisher.publish(image)
            self.rate.sleep()

if __name__ == "__main__":
    sc = SimulateCamera()

