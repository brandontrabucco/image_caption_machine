"""Author: Brandon Trabucco.
Utilities for interacting with image captions.
"""


import rospy
import numpy as np
from rt_msgs.msg import Odom
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


from image_caption_machine.msg import ImageBytes


def get_odom(target="/rt/odom"):
    """Get the current odom describing the robot.
    Returns:
        An Odom object containing positions.
    """

    try:
    	return rospy.wait_for_message(
            target, Odom, timeout=10.0)
    except Exception, e:
        rospy.logerr(str(e))

    return None


def get_camera_image(target="/camera/rgb/image_raw"):
    """Read from the camera raw topic to get an image.
    Returns:
        Numpy matrix for a single image.
    """

    image = rospy.wait_for_message(
        target, Image, timeout=10.0)
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image)

    return image


def get_bytes_msg(image):
    """Converts from numpy image to ImageBytes message
    Returns:
        ImageBytes message object
    """

    return ImageBytes(
        flat_buffer=np.reshape(image, -1).tolist(),
        height=image.shape[0], width=image.shape[1], depth=image.shape[2])


