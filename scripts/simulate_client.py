#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from image_caption_machine.srv import Caption
from image_caption_machine.msg import ImageNumpy

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class SimulateClient(object):

    def __init__(self):
        rospy.init_node("simulate_client_node")
        rospy.loginfo("Waiting for camera image.")
        message = rospy.wait_for_message(
            "camera/rgb/image_raw", Image)
        self.handle_image(message)

    def handle_image(self, image):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(image)
        rospy.loginfo("Waiting for image captioner.")
        rospy.wait_for_service('caption_service')
        try:
            caption = rospy.ServiceProxy(
                'caption_service', Caption)
            response = caption(ImageNumpy(
                flat_buffer=np.reshape(image, -1).tolist(),
                height=image.shape[0],
                width=image.shape[1],
                depth=image.shape[2]))
            rospy.loginfo("Caption was: %s", response.caption_text)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    sc = SimulateClient()

