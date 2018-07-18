#!/usr/bin/env python


"""Author: Brandon Trabucco
Client that sends camera images to Caption Server.
"""


import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge


import image_caption_machine
from image_caption_machine.srv import Caption
from image_caption_machine.srv import CaptionString
from image_caption_machine.srv import CaptionStringResponse


class CaptionClient(object):
    """Utility class to start client node and process images.
    """

    def __init__(self):
        """Start the node and listen for requests.
        """

        rospy.init_node("simulate_client")
        rospy.loginfo("Caption client started as a service.")
        self.service = rospy.Service(
            "caption_client",
            CaptionString,
            self.handle_image)
        rospy.spin()


    def handle_image(self, request):
        """Respond to a request by captioning a new image.
        """

        try:
            image = image_caption_machine.utils.get_camera_image()
            rospy.wait_for_service('caption_service', timeout=10.0)
            caption = rospy.ServiceProxy(
                'caption_service', Caption)
            response = caption(image_caption_machine.utils.get_bytes_msg(
                image)).caption_text
        except Exception, e:
            rospy.logerr(str(e))
            response = "Unknown."

        return CaptionStringResponse(
            caption_text=response)


if __name__ == "__main__":
    cc = CaptionClient()

