#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from image_caption_machine.srv import Caption
from image_caption_machine.srv import CaptionString, CaptionStringResponse
from image_caption_machine.msg import ImageNumpy

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class CaptionClient(object):

    def __init__(self):
        rospy.init_node("simulate_client_node")
        rospy.loginfo("Caption client started as a service.")
        self.service = rospy.Service(
            "caption_client",
            CaptionString,
            self.handle_image)
        rospy.spin()

    def handle_image(self, request):
        rospy.loginfo("Waiting for image from camera.")
        image = rospy.wait_for_message(
            "camera/rgb/image_raw", Image)
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(image)
        rospy.loginfo("Waiting for caption service.")
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
            return CaptionStringResponse(
                caption_text=response.caption_text)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
            return CaptionString(
                caption_text="Server error processing request.")

if __name__ == "__main__":
    cc = CaptionClient()

