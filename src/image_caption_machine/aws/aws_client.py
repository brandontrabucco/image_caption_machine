"""Author: Brandon Trabucco
Connects to AWS using POST and sends JPEG bytes.
"""


import cv2
import os.path
import rospy


from image_caption_machine.aws.aws_post import AWSPost


class AWSClient(object):
    """Utility class to initiate requests to AWS Server.
    """

    def __init__(self):
        """Set parameters to be used when connecting.
        """

        self.server_url = rospy.get_param("model_server_url")
        self.image_uri = os.path.join(os.path.dirname(
            os.path.realpath(__file__)), "image_buffer.jpg")


    def post(self, image):
        """Send an HTTP POST request to an AWS Server.
        """

        cv2.imwrite(self.image_uri, image)
        response = AWSPost(self.image_uri, self.server_url)
        if response.caption is not None:
            return response.caption[0][0].replace("<UNK>", "unknown")
        else:
            return "machine encountered a problem."
