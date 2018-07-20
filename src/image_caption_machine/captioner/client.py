"""Author: Brandon Trabucco
A utility for requesting captions.
"""


import rospy


from image_caption_machine.srv import Caption
from image_caption_machine.utils import get_bytes_msg
from image_caption_machine.utils import get_camera_image
from image_caption_machine.aws import get_captions_with_aws
from image_caption_machine.captioner.abstract import Abstract


class Client(Abstract):
    """Class for requsting image captions.
    """

    def __init__(self):
        """Build the ROS services.
        """

        rospy.wait_for_service('icm/caption', timeout=10.0)
        self.caption_service = rospy.ServiceProxy(
            "icm/caption", Caption)


    def caption(self, image=None):
        """Returns captions from an image.
        Args:
            image: image bytes string
        Returns:
            caption: str(...)
        """

        try:
            image = (get_camera_image() 
                if image is None else image)
            response = self.caption_service(get_bytes_msg(
                image)).caption_text
        except Exception, e:
            rospy.logerr(str(e))
            response = "Client encountered a problem."

        return response

