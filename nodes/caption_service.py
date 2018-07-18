#!/usr/bin/env python


"""Author: Brandon Trabucco
Server that directly requests captions from image bytes.
"""


import rospy
import numpy as np


import image_caption_machine
from image_caption_machine.srv import Caption, CaptionResponse


class CaptionService(object):
    """Service to run the image captioner utility.
    """

    def __init__(self):
        """Start listening for incoming images.
        """

        rospy.init_node("caption_service")
        rospy.loginfo("Caption service started.")
        self.service = rospy.Service(
            "caption_service", 
            Caption, 
            self.handle_caption)
        rospy.spin()


    def handle_caption(self, request):
        """Once an image arrives, send to the captioner.
        """

        image = request.image_buffer
        rospy.loginfo("Received captioning request.")
        data = np.array(image.flat_buffer).reshape(
                (image.height, image.width, image.depth))
        return CaptionResponse(image_caption_machine.get_captions(data))


if __name__ == "__main__":
    cs = CaptionService()

