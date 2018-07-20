"""Author: Brandon Trabucco
Helper functions for use in this package.
"""


import rospy
import numpy as np
from tf import TransformListener
from rt_msgs.msg import Odom
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


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


def get_pose(tf_listener, frame="base_footprint"):
    """Get the current pose describing the robot.
    Returns:
        An Pose object containing positions.
    """

    try:
	if tf_listener.canTransform(
                "map", frame, rospy.Time.now() - rospy.Duration(0.5)):

            curr_pos, curr_quat = tf_listener.lookupTransform(
                "map", frame, rospy.Time(0))

            return Pose(Point(*curr_pos), Quaternion(*curr_quat))

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
