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
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


from image_caption_machine.msg import ImageBytes


def transform_to_pose(transform):
    """Converts from Transform message into a Pose message.
    """

    return Pose(Point(transform.translation.x, 
                      transform.translation.y, 
                      transform.translation.z), transform.rotation)


def stamped_transform_to_pose(transform):
    """Converts from StampedTransform message into a Pose message.
    """

    return PoseStamped(transform.header, 
        transform_to_pose(transform.transform))


def get_pose_stamped(tf_buffer):
    """Get the current pose describing the robot.
    Returns:
        An Pose object containing positions.
    """

    while True:
        try:
            return stamped_transform_to_pose(
                tf_buffer.lookup_transform(
                    "map", "body", rospy.Time(0)))
        except Exception, e:
            rospy.logerr(str(e))


def get_image(target="/camera/rgb/image_raw"):
    """Read from the camera raw topic to get an image.
    Returns:
        Numpy matrix for a single image.
    """

    image = rospy.wait_for_message(
        target, Image, timeout=10.0)
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image)

    return image


def image_to_msg(image):
    """Converts from numpy image to ImageBytes message
    Returns:
        ImageBytes message object
    """

    return ImageBytes(
        flat_buffer=np.reshape(image, -1).tolist(),
        height=image.shape[0], width=image.shape[1], depth=image.shape[2])
