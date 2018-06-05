#!/usr/bin/env python

import rospy
from image_caption_machine.srv import Caption, CaptionResponse
from captioning_utils import run_caption

class CaptionService(object):

    def __init__(self):
        rospy.init_node("caption_service_node")
        print("Caption service started.")
        self.service = rospy.Service(
            "caption_service", 
            Caption, 
            self.handle_caption)
        rospy.spin()

    def handle_caption(self, request):
        image = request.image_buffer
        return CaptionResponse(run_caption(image))

if __name__ == "__main__":
    cs = CaptionService()