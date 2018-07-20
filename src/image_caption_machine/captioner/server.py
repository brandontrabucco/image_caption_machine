"""Author: Brandon Trabucco
A utility for requesting captions.
"""


import rospy
import numpy as np


from image_caption_machine.srv import Caption, CaptionResponse
from image_caption_machine.captioner.helper import Helper
from image_caption_machine.captioner.abstract import Abstract


class Server(Abstract):
    """Class for requsting image captions.
    """

    def __init__(self):
        """Build the ROS services.
        """

        rospy.init_node("caption_server")
        self.captioner = Helper()
        self.caption_service = rospy.Service(
            "icm/caption", Caption, self.caption)
        rospy.spin()


    def caption(self, image):
        """Returns captions from an image.
        Args:
            image: Caption() message
        Returnsi:
            captions: str(...)
        """

        image = image.image_buffer
        image = np.array(image.flat_buffer).reshape(
                (image.height, image.width, image.depth))
        return CaptionResponse(self.captioner.caption(image))
